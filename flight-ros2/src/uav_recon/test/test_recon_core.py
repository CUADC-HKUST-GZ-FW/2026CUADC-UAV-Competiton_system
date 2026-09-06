import math

from uav_recon.core import (
    CameraModel,
    Observation,
    TimedBuffer,
    Track,
    geodetic_delta_m,
    image_center_weight,
    is_empty_target_label,
    lerp_tuple,
    project_pixel_to_ground,
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
