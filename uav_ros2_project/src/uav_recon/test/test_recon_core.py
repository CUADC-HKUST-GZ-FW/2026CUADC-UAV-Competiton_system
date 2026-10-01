import math

from uav_recon.core import (
    CameraModel,
    FramePacket,
    Observation,
    PixelFramePacketManager,
    ShortPacketGapBridgeBuffer,
    TimedBuffer,
    Track,
    geodetic_delta_m,
    hermite_tuple,
    image_center_weight,
    is_empty_target_label,
    lerp_tuple,
    project_pixel_to_ground,
    propagate_geodetic_with_local_delta,
    recognition_event_to_manifest,
    resolve_packet_candidates,
)


CAMERA = CameraModel(
    fx=1000.0,
    fy=1000.0,
    cx=720.0,
    cy=540.0,
    distortion=[0.0] * 5,
    forward_tilt_deg=20.0,
    offset_flu_m=[0.0, 0.0, 0.0],
)


def test_center_pixel_projects_forward_for_identity_body_attitude():
    result = project_pixel_to_ground(
        (720.0, 540.0), (22.0, 113.0, 35.0), (0.0, 0.0, 0.0, 1.0), 0.0, CAMERA
    )
    assert abs(result.east_offset_m - 35.0 * math.tan(math.radians(20.0))) < 1e-6
    assert abs(result.north_offset_m) < 1e-6


def test_center_pixel_follows_yaw_to_north():
    half = math.radians(90.0) / 2.0
    result = project_pixel_to_ground(
        (720.0, 540.0), (22.0, 113.0, 35.0), (0.0, 0.0, math.sin(half), math.cos(half)), 0.0, CAMERA
    )
    assert abs(result.east_offset_m) < 1e-6
    assert abs(result.north_offset_m - 35.0 * math.tan(math.radians(20.0))) < 1e-6


def test_positive_left_tilt_moves_center_ray_to_body_left():
    camera = CameraModel(
        fx=1000.0,
        fy=1000.0,
        cx=720.0,
        cy=540.0,
        distortion=[0.0] * 5,
        forward_tilt_deg=12.0,
        offset_flu_m=[0.0, 0.0, 0.0],
        left_tilt_deg=4.5,
    )
    result = project_pixel_to_ground(
        (720.0, 540.0), (22.0, 113.0, 35.0), (0.0, 0.0, 0.0, 1.0), 0.0, camera
    )
    assert abs(result.east_offset_m - 35.0 * math.tan(math.radians(12.0)) / math.cos(math.radians(4.5))) < 1e-6
    assert abs(result.north_offset_m - 35.0 * math.tan(math.radians(4.5))) < 1e-6


def test_image_top_is_aircraft_forward():
    center = project_pixel_to_ground(
        (720.0, 540.0), (22.0, 113.0, 35.0), (0.0, 0.0, 0.0, 1.0), 0.0, CAMERA
    )
    image_top = project_pixel_to_ground(
        (720.0, 340.0), (22.0, 113.0, 35.0), (0.0, 0.0, 0.0, 1.0), 0.0, CAMERA
    )
    image_bottom = project_pixel_to_ground(
        (720.0, 740.0), (22.0, 113.0, 35.0), (0.0, 0.0, 0.0, 1.0), 0.0, CAMERA
    )
    assert image_top.east_offset_m > center.east_offset_m > image_bottom.east_offset_m
    assert abs(image_top.north_offset_m) < 1e-6


def test_image_left_is_aircraft_left():
    result = project_pixel_to_ground(
        (520.0, 540.0), (22.0, 113.0, 35.0), (0.0, 0.0, 0.0, 1.0), 0.0, CAMERA
    )
    assert result.north_offset_m > 0.0


def test_timed_buffer_interpolates():
    buffer = TimedBuffer()
    buffer.add(10.0, (0.0, 2.0))
    buffer.add(12.0, (2.0, 4.0))
    assert buffer.interpolate(11.0, lerp_tuple, 2.0) == (1.0, 3.0)
    assert buffer.interpolate(20.0, lerp_tuple, 2.0) is None


def test_timed_buffer_bracketed_interpolation_rejects_one_sided_sample():
    buffer = TimedBuffer()
    buffer.add(10.0, (0.0, 2.0))
    assert buffer.interpolate_bracketed(10.5, lerp_tuple, 2.0) is None
    assert not buffer.has_sample_at_or_after(10.5)

    buffer.add(12.0, (2.0, 4.0))
    assert buffer.has_sample_at_or_after(10.5)
    assert buffer.interpolate_bracketed(11.0, lerp_tuple, 2.0) == (1.0, 3.0)


def test_timed_buffer_finds_latest_anchor_at_or_before_timestamp():
    buffer = TimedBuffer()
    buffer.add(10.0, 'old')
    buffer.add(10.2, 'anchor')
    buffer.add(10.4, 'future')

    sample = buffer.latest_at_or_before(10.3, 0.11)
    assert sample is not None
    assert sample.timestamp == 10.2
    assert sample.value == 'anchor'
    assert buffer.latest_at_or_before(10.3, 0.09) is None


def test_hermite_position_interpolation_matches_constant_velocity():
    result = hermite_tuple(
        (0.0, 10.0, 2.0),
        (20.0, -2.0, 1.0),
        (2.0, 9.8, 2.1),
        (20.0, -2.0, 1.0),
        0.5,
        0.1,
    )
    assert result == (1.0, 9.9, 2.05)


def test_rtk_anchor_is_propagated_by_local_enu_delta():
    anchor = (22.0, 113.0, 35.0)
    propagated = propagate_geodetic_with_local_delta(
        anchor,
        (100.0, 200.0, 20.0),
        (105.0, 197.0, 21.5),
    )
    east, north = geodetic_delta_m(anchor[0], anchor[1], propagated[0], propagated[1])
    assert abs(east - 5.0) < 1e-6
    assert abs(north + 3.0) < 1e-6
    assert propagated[2] == 36.5


def test_track_fusion_rejects_large_outlier():
    track = Track('target_001')
    for index, east in enumerate((0.00, 0.08, -0.06, 0.04, 0.02, 12.0)):
        latitude = 22.0 + east / 111000.0
        track.add(Observation(
            timestamp=float(index), latitude=latitude, longitude=113.0, altitude_msl_m=4.0,
            label='79', class_id=79, confidence=0.98, pose_score=0.95,
            frame_path='frame.jpg', crop_path='crop.jpg'))
    fused = track.fuse(0.10)
    east, north = geodetic_delta_m(22.0, 113.0, fused['latitude'], fused['longitude'])
    assert math.hypot(east, north) < 0.15
    assert fused['observation_count'] == 5
    assert fused['label'] == '79'


def test_image_center_weight_decreases_toward_frame_edge():
    assert image_center_weight(0.0) == 1.0
    assert image_center_weight(0.5) == 0.625
    assert image_center_weight(1.0) == 0.25


def test_empty_target_label_is_case_and_whitespace_insensitive():
    assert is_empty_target_label('empty')
    assert is_empty_target_label(' EMPTY ')
    assert is_empty_target_label('blank')
    assert is_empty_target_label('\u9f98\u7a7a\u6807\u9776')
    assert not is_empty_target_label('85')
    assert not is_empty_target_label('bomber')


def test_track_fusion_prefers_center_observation():
    track = Track('target_001')
    track.add(Observation(
        timestamp=0.0, latitude=22.0, longitude=113.0, altitude_msl_m=4.0,
        label='25', class_id=25, confidence=0.98, pose_score=0.95,
        frame_path='center.jpg', crop_path='center_crop.jpg',
        center_distance_norm=0.0))
    track.add(Observation(
        timestamp=1.0, latitude=22.0 + 1.0 / 111000.0, longitude=113.0,
        altitude_msl_m=4.0, label='25', class_id=25, confidence=0.98,
        pose_score=0.95, frame_path='edge.jpg', crop_path='edge_crop.jpg',
        center_distance_norm=1.0))

    fused = track.fuse(0.10)
    _, north = geodetic_delta_m(22.0, 113.0, fused['latitude'], fused['longitude'])
    assert 0.10 < north < 0.35


def packet_observation(
    timestamp,
    frame_number,
    center_px,
    east_m=0.0,
    label='85',
):
    latitude = 22.0
    longitude = 113.0 + east_m / (
        6378137.0 * math.cos(math.radians(latitude)) * math.pi / 180.0
    )
    return Observation(
        timestamp=timestamp,
        latitude=latitude,
        longitude=longitude,
        altitude_msl_m=4.0,
        label=label,
        class_id=int(label) if label.isdigit() else 1,
        confidence=0.98,
        pose_score=0.95,
        frame_path='frame.jpg',
        crop_path='crop.jpg',
        frame_number=frame_number,
        source_sequence=frame_number,
        center_px=center_px,
    )


def fused_packet(packet_id, frame_count, east_m):
    packet = FramePacket(packet_id, '85')
    for index in range(frame_count):
        packet.add(packet_observation(
            timestamp=index / 60.0,
            frame_number=index,
            center_px=(720.0 + index, 540.0),
            east_m=east_m,
        ))
    return packet.fuse(0.10)


def test_pixel_packet_manager_uses_dynamic_gate_and_separate_group_timeout():
    manager = PixelFramePacketManager(
        gap_timeout_sec=0.20,
        pixel_gate_base_px=70.0,
        pixel_gate_rate_px_per_sec=1500.0,
        pixel_gate_max_px=220.0,
        association_gap_sec=0.10,
    )
    first = manager.add(packet_observation(1.0, 1, (100.0, 100.0)))
    same = manager.add(packet_observation(1.05, 2, (185.0, 100.0)))
    assert same is first
    assert manager.pixel_gate(0.05) == 145.0

    split = manager.add(packet_observation(1.10, 3, (390.0, 100.0)))
    assert split is not first
    assert manager.active_packet_count == 2
    assert manager.advance(1.30) == []

    groups = manager.advance(1.300001)
    assert len(groups) == 1
    assert groups[0].label == '85'
    assert [len(packet.observations) for packet in groups[0].packets] == [2, 1]


def test_pixel_packet_manager_splits_after_010_second_association_gap():
    manager = PixelFramePacketManager(
        gap_timeout_sec=0.20,
        association_gap_sec=0.10,
    )
    first = manager.add(packet_observation(1.0, 1, (100.0, 100.0)))
    split = manager.add(packet_observation(1.11, 2, (102.0, 100.0)))

    assert split is not first
    assert manager.last_assignments[0]['reason'] == 'association_gap_exceeded'
    assert manager.advance(1.21) == []
    groups = manager.advance(1.310001)
    assert len(groups) == 1
    assert [len(packet.observations) for packet in groups[0].packets] == [1, 1]


def test_relaxed_gate_keeps_60_pixel_per_frame_flight_track_together():
    manager = PixelFramePacketManager(
        gap_timeout_sec=0.20,
        association_gap_sec=0.10,
        pixel_gate_base_px=70.0,
        pixel_gate_rate_px_per_sec=1500.0,
        pixel_gate_max_px=220.0,
    )
    packets = []
    for index in range(14):
        packets.append(manager.add(packet_observation(
            1.0 + index / 60.0,
            index,
            (100.0 + 60.0 * index, 100.0),
        )))

    assert len({packet.packet_id for packet in packets}) == 1
    assert len(packets[-1].observations) == 14


def test_pixel_packet_manager_does_not_count_two_same_frame_detections_together():
    manager = PixelFramePacketManager()
    left, right = manager.add_frame([
        packet_observation(1.0, 10, (100.0, 100.0)),
        packet_observation(1.0, 10, (110.0, 100.0)),
    ])
    assert left is not right
    assert manager.active_packet_count == 2


def test_pixel_packet_manager_assigns_two_targets_one_to_one():
    manager = PixelFramePacketManager()
    first_left, first_right = manager.add_frame([
        packet_observation(1.0, 10, (100.0, 100.0)),
        packet_observation(1.0, 10, (500.0, 100.0)),
    ])
    next_right, next_left = manager.add_frame([
        packet_observation(1.02, 11, (490.0, 100.0)),
        packet_observation(1.02, 11, (110.0, 100.0)),
    ])

    assert next_left is first_left
    assert next_right is first_right
    assert len(first_left.observations) == 2
    assert len(first_right.observations) == 2


def test_pixel_packet_manager_keeps_label_flicker_in_same_track():
    manager = PixelFramePacketManager()
    first = manager.add(packet_observation(1.0, 1, (100.0, 100.0), label='85'))
    second = manager.add(packet_observation(1.02, 2, (112.0, 101.0), label='50'))
    assert second is first
    assert [item.label for item in first.observations] == ['85', '50']


def short_packet(packet_id, timestamps, centers, east_m=0.0, label='85'):
    packet = FramePacket(packet_id, label)
    for index, (timestamp, center) in enumerate(zip(timestamps, centers)):
        packet.add(packet_observation(
            timestamp=timestamp,
            frame_number=round(timestamp * 1000) + index,
            center_px=center,
            east_m=east_m,
            label=label,
        ))
    return packet


def test_short_packet_premerge_uses_time_prediction_and_ground_diameter():
    manager = PixelFramePacketManager()
    first = short_packet(
        'packet_0009',
        [1.000, 1.033, 1.083],
        [(645.0, 701.0), (656.0, 737.0), (695.0, 790.0)],
        east_m=0.0,
        label='helicopter',
    )
    second = short_packet(
        'packet_0010',
        [1.199, 1.216, 1.233, 1.250, 1.283],
        [(824.0, 914.0), (836.0, 931.0), (844.0, 949.0),
         (850.0, 970.0), (852.0, 1008.0)],
        east_m=0.5,
        label='helicopter',
    )

    packets, decisions = manager.reassemble_short_packets(
        [first, second],
        minimum_observations=8,
        max_gap_sec=0.20,
        prediction_gate_ratio=0.45,
        max_ground_diameter_m=2.0,
    )

    assert len(packets) == 1
    assert len(packets[0].observations) == 8
    assert packets[0].source_packet_ids == ['packet_0009', 'packet_0010']
    assert decisions[0]['prediction_residual_px'] <= 0.45 * decisions[0]['prediction_gate_px']
    assert decisions[0]['ground_diameter_m'] <= 2.0


def test_short_packet_premerge_can_back_predict_from_later_packet():
    manager = PixelFramePacketManager()
    first = short_packet('packet_0001', [1.0], [(100.0, 100.0)])
    second = short_packet(
        'packet_0002',
        [1.15, 1.16, 1.17, 1.18],
        [(250.0, 100.0), (260.0, 100.0), (270.0, 100.0), (280.0, 100.0)],
        east_m=0.2,
    )

    packets, decisions = manager.reassemble_short_packets(
        [first, second], 8, 0.20, 0.45, 2.0
    )

    assert len(packets) == 1
    assert len(packets[0].observations) == 5
    assert decisions[0]['prediction_direction'] == 'backward'


def test_short_packet_premerge_rejects_failed_hard_conditions():
    manager = PixelFramePacketManager()
    base = short_packet(
        'packet_0001',
        [1.00, 1.02, 1.04],
        [(100.0, 100.0), (120.0, 100.0), (140.0, 100.0)],
    )
    late = short_packet(
        'packet_0002',
        [1.25, 1.27],
        [(350.0, 100.0), (370.0, 100.0)],
    )
    bad_prediction = short_packet(
        'packet_0003',
        [1.15, 1.17],
        [(500.0, 500.0), (520.0, 500.0)],
    )
    far_ground = short_packet(
        'packet_0004',
        [1.15, 1.17],
        [(250.0, 100.0), (270.0, 100.0)],
        east_m=2.1,
    )

    for candidate in (late, bad_prediction, far_ground):
        packets, decisions = manager.reassemble_short_packets(
            [base, candidate], 8, 0.20, 0.45, 2.0
        )
        assert len(packets) == 2
        assert decisions == []


def test_short_packet_premerge_does_not_change_existing_final_packet():
    manager = PixelFramePacketManager()
    final_packet = short_packet(
        'packet_0001',
        [1.0 + index / 60.0 for index in range(8)],
        [(100.0 + 10.0 * index, 100.0) for index in range(8)],
    )
    continuation = short_packet(
        'packet_0002',
        [1.20, 1.22, 1.24],
        [(220.0, 100.0), (230.0, 100.0), (240.0, 100.0)],
    )

    packets, decisions = manager.reassemble_short_packets(
        [final_packet, continuation], 8, 0.20, 0.45, 2.0
    )

    assert len(packets) == 2
    assert decisions == []
    assert len(final_packet.observations) == 8


def test_gap_bridge_recovers_real_0929_five_plus_five_detector_hole():
    manager = PixelFramePacketManager()
    bridge = ShortPacketGapBridgeBuffer(
        manager,
        minimum_observations=8,
        max_gap_sec=0.85,
        retention_sec=1.20,
        minimum_fragment_observations=3,
        minimum_direction_cosine=0.75,
        max_speed_px_per_sec=1800.0,
        max_ground_diameter_m=2.0,
    )
    first = short_packet(
        'packet_0003',
        [0.00000, 0.01650, 0.03301, 0.08265, 0.18160],
        [(623.44, 62.26), (631.88, 74.77), (637.03, 88.43),
         (653.09, 126.94), (697.73, 209.27)],
        east_m=0.0,
    )
    second = short_packet(
        'packet_0004',
        [0.90803, 0.92455, 0.94110, 0.99056, 1.00709],
        [(855.47, 903.05), (854.30, 922.03), (852.19, 940.55),
         (837.66, 997.27), (835.08, 1016.72)],
        east_m=0.4,
    )

    ready, expired, decisions = bridge.offer([first], first.last_timestamp)
    assert ready == []
    assert expired == []
    assert decisions == []
    assert bridge.pending_packet_count == 1

    ready, expired, decisions = bridge.offer([second], second.last_timestamp)
    assert expired == []
    assert bridge.pending_packet_count == 0
    assert len(ready) == 1
    assert len(ready[0].observations) == 10
    assert ready[0].source_packet_ids == ['packet_0003', 'packet_0004']
    assert decisions[0]['action'] == 'bridge_detector_gap'
    assert abs(decisions[0]['gap_sec'] - 0.72643) < 1e-5
    assert decisions[0]['minimum_direction_cosine'] > 0.90
    assert decisions[0]['speed_px_per_sec'] < 1000.0
    assert decisions[0]['ground_diameter_m'] <= 2.0


def test_gap_bridge_rejects_far_ground_or_reversed_motion():
    manager = PixelFramePacketManager()
    first = short_packet(
        'packet_0001',
        [1.00, 1.03, 1.06],
        [(100.0, 100.0), (110.0, 130.0), (120.0, 160.0)],
    )
    far_ground = short_packet(
        'packet_0002',
        [1.50, 1.53, 1.56],
        [(180.0, 500.0), (190.0, 530.0), (200.0, 560.0)],
        east_m=2.1,
    )
    reversed_motion = short_packet(
        'packet_0003',
        [1.50, 1.53, 1.56],
        [(180.0, 500.0), (170.0, 470.0), (160.0, 440.0)],
    )

    packets, decisions = manager.bridge_short_packet_gaps(
        [first, far_ground], 8, 0.85, 3, 0.75, 1800.0, 2.0
    )
    assert len(packets) == 2
    assert decisions == []

    packets, decisions = manager.bridge_short_packet_gaps(
        [first, reversed_motion], 8, 0.85, 3, 0.75, 1800.0, 2.0
    )
    assert len(packets) == 2
    assert decisions == []


def test_gap_bridge_expires_unmatched_short_packet_without_finalizing_it():
    manager = PixelFramePacketManager()
    bridge = ShortPacketGapBridgeBuffer(manager, 8, retention_sec=1.20)
    packet = short_packet(
        'packet_0001',
        [2.00, 2.03, 2.06],
        [(100.0, 100.0), (110.0, 120.0), (120.0, 140.0)],
    )
    ready, expired, decisions = bridge.offer([packet], 2.06)
    assert ready == []
    assert expired == []
    assert decisions == []

    expired = bridge.expire(3.260001)
    assert expired == [packet]
    assert bridge.pending_packet_count == 0


def test_pixel_packet_manager_forces_group_close_at_frame_limit():
    manager = PixelFramePacketManager(max_observations=3)
    for index in range(3):
        manager.add(packet_observation(
            1.0 + index * 0.01,
            index,
            (100.0 + index, 100.0),
        ))

    groups = manager.take_limit_reached_group()
    assert len(groups) == 1
    assert groups[0].closure_reason == 'max_observations'
    assert len(groups[0].packets) == 1
    assert len(groups[0].packets[0].observations) == 3
    assert manager.active_packet_count == 0
    assert manager.pending_packet_count == 0

    replacement = manager.add(packet_observation(1.04, 4, (104.0, 100.0)))
    assert len(replacement.observations) == 1


def test_frame_packet_uses_valid_frame_count_majority_for_label():
    packet = FramePacket('packet_0001', '85')
    labels = ['85'] * 7 + ['50'] * 4
    for index, label in enumerate(labels):
        packet.add(packet_observation(
            timestamp=index / 60.0,
            frame_number=index,
            center_px=(720.0 + index, 540.0),
            label=label,
        ))
    fused = packet.fuse(0.10)
    assert fused['label'] == '85'
    assert fused['class_id'] == 85
    assert fused['label_counts'] == {'85': 7, '50': 4}
    assert abs(fused['label_consensus'] - 7.0 / 11.0) < 1e-9


def test_frame_packet_tracks_all_frames_but_fuses_only_valid_geolocations():
    packet = FramePacket('packet_0001', 'mixed')
    labels = ['50'] * 5 + ['85'] * 7
    for index, label in enumerate(labels):
        observation = packet_observation(
            timestamp=index / 60.0,
            frame_number=index,
            center_px=(720.0 + index, 540.0),
            label=label,
        )
        if index >= 6:
            observation.latitude = math.nan
            observation.longitude = math.nan
            observation.altitude_msl_m = math.nan
            observation.geolocation_resolved = True
            observation.geolocation_valid = False
            observation.geolocation_error = 'telemetry_gap'
        packet.add(observation)

    fused = packet.fuse(0.10)
    assert fused['label'] == '85'
    assert fused['label_counts'] == {'50': 5, '85': 7}
    assert fused['tracking_observation_count'] == 12
    assert fused['geolocated_observation_count'] == 6
    assert fused['telemetry_rejected_observation_count'] == 6


def test_recognition_event_stream_converts_every_valid_detection():
    manifest = recognition_event_to_manifest({
        'record_type': 'recognition',
        'det_found': True,
        'frame': 42,
        'capture_timestamp_unix_ns': 123000000000,
        'capture_monotonic_ns': 456000000000,
        'capture_clock_source': 'camera',
        'source_sequence': 99,
        'frame_width': 1440,
        'frame_height': 1080,
        'mode': 'digit',
        'detections': [
            {
                'detection_index': 8,
                'score': 0.90,
                'roi': [500, 300, 100, 100],
                'center': [550.0, 350.0],
                'class_id': 85,
                'class_prob': 0.95,
                'class_label': '85',
            },
            {
                'detection_index': 3,
                'score': 0.92,
                'roi': [100, 100, 100, 100],
                'center': [150.0, 150.0],
                'class_id': 50,
                'class_prob': 0.96,
                'class_label': '50',
            },
        ],
    })

    assert manifest['frame'] == 42
    assert manifest['source_sequence'] == 99
    assert manifest['count'] == 2
    assert [crop['class_label'] for crop in manifest['crops']] == ['50', '85']
    assert [crop['src'] for crop in manifest['crops']] == ['crop_00.jpg', 'crop_01.jpg']


def test_packet_candidates_merge_with_frame_count_weight_inside_15m():
    stronger = fused_packet('packet_0001', 20, 0.0)
    weaker = fused_packet('packet_0002', 10, 1.2)
    winners, decisions = resolve_packet_candidates([weaker, stronger], 1.5, 10.0)
    assert len(winners) == 1
    assert winners[0]['observation_count'] == 30
    east, _ = geodetic_delta_m(
        22.0, 113.0, winners[0]['latitude'], winners[0]['longitude']
    )
    assert abs(east - 0.4) < 0.01
    assert decisions[-1]['action'] == 'merge'


def test_packet_candidates_discard_smaller_between_15m_and_10m():
    stronger = fused_packet('packet_0001', 20, 0.0)
    weaker = fused_packet('packet_0002', 12, 4.0)
    winners, decisions = resolve_packet_candidates([weaker, stronger], 1.5, 10.0)
    assert len(winners) == 1
    assert winners[0]['packet_ids'] == ['packet_0001']
    assert decisions[-1]['action'] == 'discard_smaller_packet'


def test_packet_candidates_keep_coordinates_at_least_10m_apart():
    first = fused_packet('packet_0001', 20, 0.0)
    second = fused_packet('packet_0002', 12, 10.1)
    winners, _ = resolve_packet_candidates([second, first], 1.5, 10.0)
    assert len(winners) == 2


def test_merged_packet_candidates_revote_all_frame_labels():
    left = FramePacket('packet_0001', '10')
    right = FramePacket('packet_0002', '30')
    for index, label in enumerate(['10'] * 6 + ['20'] * 5):
        left.add(packet_observation(
            timestamp=index / 60.0,
            frame_number=index,
            center_px=(700.0 + index, 540.0),
            east_m=0.0,
            label=label,
        ))
    for index, label in enumerate(['30'] * 6 + ['20'] * 5):
        right.add(packet_observation(
            timestamp=1.0 + index / 60.0,
            frame_number=100 + index,
            center_px=(900.0 + index, 540.0),
            east_m=1.0,
            label=label,
        ))

    winners, decisions = resolve_packet_candidates(
        [left.fuse(0.10), right.fuse(0.10)],
        1.5,
        10.0,
    )
    assert len(winners) == 1
    assert winners[0]['label'] == '20'
    assert winners[0]['class_id'] == 20
    assert winners[0]['label_counts'] == {'10': 6, '20': 10, '30': 6}
    assert decisions[-1]['action'] == 'merge'
