"""A-B(virtual)-U-R-C(virtual)-D geometry and mission safety tests."""

import asyncio
import math
import threading
import time
from collections import deque
from types import SimpleNamespace

import pytest

from uav_fcu_interface.fcu_interface_mavros_node import FcuInterfaceMavrosNode


def make_node_without_ros():
    node = FcuInterfaceMavrosNode.__new__(FcuInterfaceMavrosNode)
    node.a_offset_m = 160.0
    node.b_offset_m = 95.0
    node.u_offset_m = 75.0
    node.release_offset_m = 56.0
    node.d_offset_m = 160.0
    node.a_acceptance_radius_m = 30.0
    node.b_acceptance_radius_m = 15.0
    node.c_acceptance_radius_m = 8.0
    node.d_acceptance_radius_m = 30.0
    node.mission_altitude_m = 35.0
    node.control_mode = 'DRY_RUN'
    node.enable_real_payload_release = False
    node.servo_channel = 7
    node.safe_pwm = 1350
    node.release_pwm = 1900
    node.release_command_enabled = False
    node.b_check_half_width_m = 12.0
    node.b_check_length_m = 60.0
    node.b_check_end_distance_from_a_m = 50.0
    node.b_check_required_samples = 3
    node.b_check_max_heading_error_deg = 20.0
    node.b_check_convergence_tolerance_m = 1.5
    node.b_check_min_converging_ratio = 0.5
    node.gps_stale_timeout_sec = 1.0
    node.gps_history = deque(maxlen=20)
    node._last_b_check_pending_log_time = 0.0
    node._gps_lock = threading.Lock()
    node._dynamic_state_lock = threading.Lock()
    node._mission_update_lock = threading.Lock()
    node.composite_completion_reported = False
    node.composite_final_seq = None
    node.composite_total_count = 0
    node.composite_seq_a = None
    node.composite_seq_u = None
    node.composite_seq_r = None
    node.composite_seq_release = None
    node.composite_seq_d = None
    node.composite_route_points = None
    node.dynamic_indices = {}
    node.composite_expected_waypoints = None
    node.composite_safe_r_waypoint = None
    node.current_waypoints = None
    node.current_gps = None
    node.current_raw_gps = None
    node.current_vfr_hud = None
    node._last_mission_current_seq = None
    node._b_previous_signed_m = None
    node._b_crossing_triggered = False
    node._dynamic_update_started = False
    node._dynamic_update_verified_monotonic = None
    node._dynamic_update_verified_gps = None
    node._dynamic_update_result = 'NOT_OBSERVED'
    node._r_active_before_verify_reported = False
    node.dynamic_r_enabled = False
    node.dynamic_r_test_mode = False
    node.dynamic_r_test_offset_m = 50.0
    node.dynamic_r_update_timeout_sec = 6.0
    node.aburcd_update_metrics_enabled = True
    node.allow_mission_upload = True
    node.composite_ab_check_result = None
    node.composite_ab_check_state = 'WAIT_A'
    node.composite_c_confirmed = False
    node.composite_c_confirmation = 'none'
    node.composite_c_min_distance_m = float('inf')
    node.mission_summary_event_publisher = SimpleNamespace(
        publish=lambda _message: None
    )
    return node


def prepare_dynamic_update_node():
    node = make_node_without_ros()
    node.active_task_id = 'target_001'
    node.mission_type = 'COMPOSITE'
    node.get_logger = lambda: NullLogger()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    original = [
        node.make_nav_waypoint(22.8800 + index * 0.001, 113.4900, 35.0, 15.0)
        for index in range(14)
    ]
    mission, _ = node.build_composite_mission(points, original, 5, 9)
    node.composite_route_points = points
    node.composite_expected_waypoints = [
        node.clone_waypoint(wp) for wp in mission
    ]
    node.current_waypoints = SimpleNamespace(
        current_seq=node.composite_seq_u, waypoints=mission,
    )
    node.dynamic_r_enabled = True
    node.dynamic_r_test_mode = True
    events = []
    node.publish_mission_summary_event = (
        lambda event, **fields: events.append((event, fields))
    )
    return node, points, mission, events


class NullLogger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass


@pytest.mark.parametrize('heading_deg', [0.0, 90.0, 180.0, 270.0])
def test_abcdr_distances_and_order(heading_deg):
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, heading_deg)

    assert points['C'] == {'lat': 22.8848, 'lon': 113.4956}
    expected_distances = {
        'A': 160.0,
        'B': 95.0,
        'U': 75.0,
        'R': 56.0,
        'D': 160.0,
    }
    for name, expected_distance_m in expected_distances.items():
        distance_m = node.distance_m(
            points[name]['lat'],
            points[name]['lon'],
            22.8848,
            113.4956,
        )
        assert distance_m == pytest.approx(expected_distance_m, abs=0.05)
    assert node.distance_m(
        points['A']['lat'],
        points['A']['lon'],
        points['B']['lat'],
        points['B']['lon'],
    ) == pytest.approx(65.0, abs=0.05)
    assert node.distance_m(
        points['B']['lat'],
        points['B']['lon'],
        points['U']['lat'],
        points['U']['lon'],
    ) == pytest.approx(20.0, abs=0.05)
    assert node.distance_m(
        points['B']['lat'],
        points['B']['lon'],
        points['R']['lat'],
        points['R']['lon'],
    ) == pytest.approx(39.0, abs=0.05)
    assert math.isfinite(points['D']['lat']) and math.isfinite(points['D']['lon'])


@pytest.mark.parametrize(
    'control_mode,enabled,channel,safe_pwm,release_pwm',
    [
        ('DRY_RUN', True, 7, 1350, 1900),
        ('REAL_CONTROL', False, 7, 1350, 1900),
        ('REAL_CONTROL', True, 0, 1350, 1900),
        ('REAL_CONTROL', True, 7, 700, 1900),
        ('REAL_CONTROL', True, 7, 1350, 2300),
        ('REAL_CONTROL', True, 7, 1900, 1900),
    ],
)
def test_release_gate_rejects_unsafe_configuration(
    control_mode, enabled, channel, safe_pwm, release_pwm
):
    node = make_node_without_ros()
    node.control_mode = control_mode
    node.enable_real_payload_release = enabled
    node.servo_channel = channel
    node.safe_pwm = safe_pwm
    node.release_pwm = release_pwm
    allowed, reason = node.can_insert_release_command()
    assert not allowed
    assert reason


def test_b_approach_requires_multiple_forward_samples_and_ignores_old_anomaly():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    now = time.time()
    bad_lat, bad_lon = node.destination_point(
        points['A']['lat'], points['A']['lon'], 0.0, 30.0
    )
    node.gps_history.append((now - 0.4, bad_lat, bad_lon))
    for index, distance_m in enumerate((20.0, 35.0, 50.0)):
        lat, lon = node.destination_point(
            points['A']['lat'], points['A']['lon'], 90.0, distance_m
        )
        node.gps_history.append((now - 0.3 + index * 0.1, lat, lon))
    gps = SimpleNamespace(
        latitude=node.gps_history[-1][1],
        longitude=node.gps_history[-1][2],
        status=SimpleNamespace(status=0),
    )

    passed, reason, metrics = node.check_b_approach(
        gps, points['A'], points['B'], 90.0
    )
    assert passed, reason
    assert metrics['sample_count'] == 3
    assert metrics['forward_progress_m'] > 1.0
    assert abs(metrics['heading_error_deg']) < 1.0
    assert metrics['cross_track_m'] < 1.0
    assert metrics['converging_ratio'] == 1.0


def append_ab_samples(node, a_point, heading_deg, forward_offsets, lateral_offsets):
    now = time.time()
    for index, (forward_m, lateral_m) in enumerate(
        zip(forward_offsets, lateral_offsets)
    ):
        point = node.destination_point(
            a_point['lat'], a_point['lon'], heading_deg, forward_m
        )
        lateral_point = node.destination_point(
            point[0], point[1], heading_deg + 90.0, lateral_m
        )
        node.gps_history.append(
            (now - 0.3 + index * 0.1, lateral_point[0], lateral_point[1])
        )
    return SimpleNamespace(
        latitude=node.gps_history[-1][1],
        longitude=node.gps_history[-1][2],
        status=SimpleNamespace(status=0),
    )


def test_b_approach_accepts_heading_cross_track_and_converging_trend():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    gps = append_ab_samples(
        node, points['A'], 90.0, (15.0, 30.0, 45.0), (10.0, 6.0, 2.0)
    )

    passed, reason, metrics = node.check_b_approach(
        gps, points['A'], points['B'], 90.0
    )

    assert passed, reason
    assert abs(metrics['heading_error_deg']) <= node.b_check_max_heading_error_deg
    assert metrics['cross_track_m'] <= node.b_check_half_width_m
    assert metrics['convergence_slope_m_per_sample'] < 0.0
    assert metrics['converging_ratio'] == 1.0


def test_b_approach_rejects_heading_error_against_ab_line():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    gps = append_ab_samples(
        node, points['A'], 90.0, (15.0, 30.0, 45.0), (40.0, 20.0, 0.0)
    )

    passed, reason, metrics = node.check_b_approach(
        gps, points['A'], points['B'], 90.0
    )

    assert not passed
    assert reason == 'heading_error_exceeded'
    assert abs(metrics['heading_error_deg']) > node.b_check_max_heading_error_deg


def test_b_approach_rejects_current_cross_track_error():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    gps = append_ab_samples(
        node, points['A'], 90.0, (15.0, 30.0, 45.0), (20.0, 20.0, 20.0)
    )

    passed, reason, metrics = node.check_b_approach(
        gps, points['A'], points['B'], 90.0
    )

    assert not passed
    assert reason == 'cross_track_exceeded'
    assert metrics['cross_track_m'] > node.b_check_half_width_m


def test_b_approach_rejects_multi_point_diverging_trend():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    gps = append_ab_samples(
        node, points['A'], 90.0, (15.0, 30.0, 45.0), (2.0, 6.0, 10.0)
    )

    passed, reason, metrics = node.check_b_approach(
        gps, points['A'], points['B'], 90.0
    )

    assert not passed
    assert reason == 'cross_track_not_converging'
    assert metrics['cross_track_m'] <= node.b_check_half_width_m
    assert metrics['convergence_slope_m_per_sample'] > 0.0


def test_b_approach_single_sample_collects_instead_of_immediate_failure():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    lat, lon = node.destination_point(
        points['A']['lat'], points['A']['lon'], 90.0, 40.0
    )
    node.gps_history.append((time.time(), lat, lon))
    gps = SimpleNamespace(
        latitude=lat, longitude=lon, status=SimpleNamespace(status=0)
    )
    passed, reason, metrics = node.check_b_approach(
        gps, points['A'], points['B'], 90.0
    )
    assert passed is None
    assert reason == 'gps_history_insufficient'
    assert metrics['sample_count'] == 1


def test_empty_gps_history_is_pending_not_failure():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    gps = SimpleNamespace(
        latitude=points['B']['lat'],
        longitude=points['B']['lon'],
        status=SimpleNamespace(status=0),
    )
    passed, reason, metrics = node.check_b_approach(
        gps, points['A'], points['B'], 90.0
    )
    assert passed is None
    assert reason == 'gps_history_empty'
    assert metrics['sample_count'] == 0


def test_reset_b_check_window_clears_composite_samples():
    node = make_node_without_ros()
    node.gps_history.append((time.time(), 22.0, 113.0))
    node.reset_b_check_window()
    assert list(node.gps_history) == []


def test_home_altitude_difference_is_ignored():
    node = make_node_without_ros()
    expected = [
        node.make_nav_waypoint(22.8800, 113.4900, 0.10, 0.0),
        node.make_nav_waypoint(22.8810, 113.4910, 35.0, 15.0),
    ]
    actual = [node.clone_waypoint(wp) for wp in expected]
    actual[0].z_alt = -0.09
    assert node.verify_temporary_mission(expected, actual)[0]


def test_non_home_altitude_difference_still_fails():
    node = make_node_without_ros()
    expected = [
        node.make_nav_waypoint(22.8800, 113.4900, 0.10, 0.0),
        node.make_nav_waypoint(22.8810, 113.4910, 35.0, 15.0),
    ]
    actual = [node.clone_waypoint(wp) for wp in expected]
    actual[1].z_alt += 0.21
    ok, reason = node.verify_temporary_mission(expected, actual)
    assert not ok
    assert 'seq=1 field=z_alt' in reason


def test_home_non_altitude_mismatch_still_fails():
    node = make_node_without_ros()
    expected = [
        node.make_nav_waypoint(22.8800, 113.4900, 0.10, 0.0),
        node.make_nav_waypoint(22.8810, 113.4910, 35.0, 15.0),
    ]
    actual = [node.clone_waypoint(wp) for wp in expected]
    actual[0].x_lat += 1.0e-5
    ok, reason = node.verify_temporary_mission(expected, actual)
    assert not ok
    assert 'seq=0 field=x_lat' in reason


def test_composite_upload_requires_auto_without_changing_mode():
    node = make_node_without_ros()
    node.active_task_id = 'target_001'
    node.log_state = 'PLANNING'
    node.mission_type = 'ORIGINAL'
    node.insert_wp_index = 5
    node.resume_wp_index = 8
    node.clear_mission_before_full_push = False
    node.get_logger = lambda: NullLogger()
    original = [
        node.make_nav_waypoint(22.8800 + index * 0.001, 113.4900, 35.0, 15.0)
        for index in range(12)
    ]
    original[0].z_alt = 0.0
    pushed = []
    pull_count = 0

    async def pull_mission_async():
        nonlocal pull_count
        pull_count += 1
        if pull_count == 1:
            return SimpleNamespace(waypoints=original, current_seq=2)
        return SimpleNamespace(waypoints=pushed, current_seq=2)

    async def push_mission_async(waypoints):
        pushed.extend(node.clone_waypoint(wp) for wp in waypoints)

    current_calls = []

    async def set_current_mission_item_async(seq):
        current_calls.append(seq)

    modes = []
    node.pull_mission_async = pull_mission_async
    node.push_mission_async = push_mission_async
    node.set_current_mission_item_async = set_current_mission_item_async
    node.wait_for_current_seq = lambda seq, timeout_sec: True
    node.current_mode = lambda: 'AUTO'
    node.set_mode = lambda mode: modes.append(mode)
    node.publish_mission_summary_event = lambda *_args, **_kwargs: None
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)

    message = asyncio.run(node.handle_goto_global_composite_async(points))

    assert modes == []
    assert current_calls == [2]
    assert node.composite_final_seq == node.composite_seq_d
    assert node.composite_final_seq < len(pushed) - 1
    assert 'AUTO remained confirmed without an automatic mode change' in message


def test_composite_aurd_layout_keeps_b_and_c_virtual():
    node = make_node_without_ros()
    node.release_command_enabled = True
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    original = [
        node.make_nav_waypoint(22.8800 + index * 0.001, 113.4900, 35.0, 15.0)
        for index in range(14)
    ]

    mission, a_seq = node.build_composite_mission(points, original, 5, 9)

    assert len(mission) == 15
    assert a_seq == node.composite_seq_a == 5
    assert node.composite_seq_u == 6
    assert node.composite_seq_r == 7
    assert node.composite_seq_release == 8
    assert node.composite_seq_d == 9
    assert node.dynamic_indices == {
        'A': 5, 'U': 6, 'R': 7, 'RELEASE': 8, 'D': 9, 'RESUME': 10,
    }
    assert [wp.command for wp in mission[5:10]] == [
        node.MAV_CMD_NAV_WAYPOINT,
        node.MAV_CMD_NAV_WAYPOINT,
        node.MAV_CMD_NAV_WAYPOINT,
        node.MAV_CMD_DO_SET_SERVO,
        node.MAV_CMD_NAV_WAYPOINT,
    ]
    assert mission[5].x_lat == points['A']['lat']
    assert mission[6].x_lat == points['U']['lat']
    assert mission[7].x_lat == points['R']['lat']
    assert mission[9].x_lat == points['D']['lat']
    assert mission[10].x_lat == original[9].x_lat
    assert points['B'] and points['C']
    assert all(wp.x_lat != points['B']['lat'] for wp in mission[5:10])
    assert all(wp.x_lat != points['C']['lat'] for wp in mission[5:10])


def test_dynamic_r_requires_explicit_test_mode_and_stays_between_u_and_c():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    node.composite_route_points = points
    snapshot = {'ground_speed_mps': 20.0, 'altitude_m': 35.0}

    node.dynamic_r_enabled = True
    candidate = node.compute_dynamic_r(snapshot, points['C'])
    assert not candidate['valid']
    assert candidate['reason'] == 'production_dynamic_r_formula_not_configured'

    node.dynamic_r_test_mode = True
    candidate = node.compute_dynamic_r(snapshot, points['C'])
    assert candidate['valid']
    assert candidate['reason'] == 'TEST_ONLY_fixed_offset'
    assert node.validate_dynamic_r_candidate(candidate)[0]
    assert node.distance_m(
        candidate['lat'], candidate['lon'],
        points['C']['lat'], points['C']['lon'],
    ) == pytest.approx(50.0, abs=0.05)


def test_dynamic_r_outside_u_c_interval_is_rejected():
    node = make_node_without_ros()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    node.composite_route_points = points
    bad_lat, bad_lon = node.destination_point(
        points['C']['lat'], points['C']['lon'],
        points['reverse_heading_deg'], 90.0,
    )
    ok, reason = node.validate_dynamic_r_candidate({
        'valid': True, 'lat': bad_lat, 'lon': bad_lon, 'alt': 35.0,
    })
    assert not ok
    assert 'R_DYNAMIC_OUT_OF_RANGE' in reason


def test_virtual_b_crossing_triggers_once_only_while_u_is_current():
    node = make_node_without_ros()
    node.dynamic_r_enabled = True
    node.mission_type = 'COMPOSITE'
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    node.composite_route_points = points
    node.dynamic_indices = {'A': 5, 'U': 6, 'R': 7, 'RELEASE': 8, 'D': 9}
    node.composite_seq_a = 5
    node.composite_seq_u = 6
    node.composite_seq_r = 7
    node.composite_seq_release = 8
    node.composite_seq_d = 9
    node.current_waypoints = SimpleNamespace(
        current_seq=6, waypoints=[object()] * 12
    )
    node.current_vfr_hud = SimpleNamespace(
        groundspeed=21.0, climb=-0.2, heading=90
    )
    node.get_logger = lambda: NullLogger()
    events = []
    node.publish_mission_summary_event = (
        lambda event, **fields: events.append((event, fields))
    )
    snapshots = []
    node._dynamic_r_worker = lambda snapshot: snapshots.append(snapshot)

    before_lat, before_lon = node.destination_point(
        points['B']['lat'], points['B']['lon'], 270.0, 2.0
    )
    after_lat, after_lon = node.destination_point(
        points['B']['lat'], points['B']['lon'], 90.0, 2.0
    )
    before = SimpleNamespace(
        latitude=before_lat, longitude=before_lon, altitude=35.0,
        status=SimpleNamespace(status=0),
    )
    after = SimpleNamespace(
        latitude=after_lat, longitude=after_lon, altitude=35.0,
        status=SimpleNamespace(status=0),
    )
    node.current_gps = before
    node._maybe_detect_virtual_b_crossing(before)
    node.current_gps = after
    node._maybe_detect_virtual_b_crossing(after)
    node._maybe_detect_virtual_b_crossing(after)
    deadline = time.monotonic() + 1.0
    while not snapshots and time.monotonic() < deadline:
        time.sleep(0.001)

    assert [name for name, _ in events] == ['b_crossed', 'b_state_frozen']
    assert len(snapshots) == 1
    assert snapshots[0]['current_seq'] == 6
    assert snapshots[0]['ground_speed_mps'] == 21.0


def test_dynamic_r_update_uses_one_waypoint_and_pullback_verification():
    node = make_node_without_ros()
    node.mission_type = 'COMPOSITE'
    node.get_logger = lambda: NullLogger()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    original = [
        node.make_nav_waypoint(22.8800 + index * 0.001, 113.4900, 35.0, 15.0)
        for index in range(14)
    ]
    mission, _ = node.build_composite_mission(points, original, 5, 9)
    node.composite_route_points = points
    node.composite_expected_waypoints = [
        node.clone_waypoint(wp) for wp in mission
    ]
    node.current_waypoints = SimpleNamespace(
        current_seq=node.composite_seq_u, waypoints=mission,
    )
    node.dynamic_r_enabled = True
    node.dynamic_r_test_mode = True
    events = []
    node.publish_mission_summary_event = (
        lambda event, **fields: events.append((event, fields))
    )
    partial_calls = []

    async def partial(start_index, waypoint):
        partial_calls.append((start_index, node.clone_waypoint(waypoint)))

    async def pull():
        updated = [node.clone_waypoint(wp) for wp in mission]
        updated[partial_calls[0][0]] = node.clone_waypoint(partial_calls[0][1])
        return SimpleNamespace(
            current_seq=node.composite_seq_u,
            waypoints=updated,
        )

    node.push_partial_mission_async = partial
    node.pull_mission_async = pull
    candidate = node.compute_dynamic_r({'ground_speed_mps': 20.0}, points['C'])
    snapshot = {'timestamp_monotonic': time.monotonic(), 'current_seq': 6}
    asyncio.run(node._update_dynamic_r_async(candidate, snapshot, 0.1))

    assert len(partial_calls) == 1
    assert partial_calls[0][0] == node.composite_seq_r
    assert node._dynamic_update_result == 'DYNAMIC_R'
    assert 'r_push_verified' in [name for name, _ in events]


def test_dynamic_r_update_rejects_late_without_push():
    node = make_node_without_ros()
    node.get_logger = lambda: NullLogger()
    node.dynamic_indices = {'A': 5, 'U': 6, 'R': 7, 'RELEASE': 8, 'D': 9}
    node.composite_seq_r = 7
    node.current_waypoints = SimpleNamespace(
        current_seq=7, waypoints=[object()] * 12
    )
    node.composite_route_points = node.compute_abcdr_points(
        22.8848, 113.4956, 90.0
    )
    events = []
    node.publish_mission_summary_event = (
        lambda event, **fields: events.append(event)
    )
    calls = []
    node.push_partial_mission_async = lambda *_args: calls.append(True)
    candidate = {
        'valid': True, 'lat': 22.0, 'lon': 113.0, 'alt': 35.0,
    }
    asyncio.run(node._update_dynamic_r_async(
        candidate, {'timestamp_monotonic': time.monotonic()}, 0.1
    ))
    assert calls == []
    assert node._dynamic_update_result == 'UPDATE_TOO_LATE'
    assert 'r_update_rejected_late' in events


def test_dynamic_r_push_failure_keeps_safe_mission_cache():
    node, points, mission, events = prepare_dynamic_update_node()

    async def failing_push(_seq, _waypoint):
        raise RuntimeError('injected partial push failure')

    node.push_partial_mission_async = failing_push
    snapshot = {'timestamp_monotonic': time.monotonic(), 'current_seq': 6}
    node._dynamic_r_worker(snapshot)

    assert node._dynamic_update_result == 'UPDATE_FAILED'
    assert node.composite_expected_waypoints[node.composite_seq_r].x_lat == (
        mission[node.composite_seq_r].x_lat
    )
    assert any(
        name == 'r_push_failed' and 'injected' in fields['failure_reason']
        for name, fields in events
    )


def test_dynamic_r_verify_failure_performs_one_safe_r_restore():
    node, points, mission, events = prepare_dynamic_update_node()
    partial_calls = []
    pull_count = 0

    async def partial(seq, waypoint):
        partial_calls.append((seq, node.clone_waypoint(waypoint)))

    async def pull():
        nonlocal pull_count
        pull_count += 1
        if pull_count == 1:
            corrupt = [node.clone_waypoint(wp) for wp in mission]
            corrupt[node.composite_seq_d].x_lat += 0.01
            return SimpleNamespace(
                current_seq=node.composite_seq_u, waypoints=corrupt,
            )
        restored = [node.clone_waypoint(wp) for wp in mission]
        return SimpleNamespace(
            current_seq=node.composite_seq_u, waypoints=restored,
        )

    node.push_partial_mission_async = partial
    node.pull_mission_async = pull
    candidate = node.compute_dynamic_r({'ground_speed_mps': 20.0}, points['C'])
    asyncio.run(node._update_dynamic_r_async(
        candidate, {'timestamp_monotonic': time.monotonic()}, 0.1
    ))

    assert len(partial_calls) == 2
    assert partial_calls[1][0] == node.composite_seq_r
    assert partial_calls[1][1].x_lat == mission[node.composite_seq_r].x_lat
    assert node._dynamic_update_result == 'R_SAFE_FALLBACK'
    assert any(name == 'r_push_verify_failed' for name, _ in events)


def test_dynamic_r_timeout_stops_before_partial_push():
    node, points, _mission, events = prepare_dynamic_update_node()
    node.dynamic_r_update_timeout_sec = 0.001
    original_compute = node.compute_dynamic_r

    def slow_compute(snapshot, target):
        time.sleep(0.005)
        return original_compute(snapshot, target)

    node.compute_dynamic_r = slow_compute
    calls = []
    node.push_partial_mission_async = lambda *_args: calls.append(True)
    node._dynamic_r_worker({
        'timestamp_monotonic': time.monotonic(), 'current_seq': 6,
    })

    assert calls == []
    assert node._dynamic_update_result == 'UPDATE_FAILED'
    assert any(
        name == 'r_push_failed'
        and fields['failure_reason'] == 'dynamic_r_update_timeout_before_push'
        for name, fields in events
    )


def test_dynamic_r_worker_rejects_busy_mission_transaction():
    node, _points, _mission, events = prepare_dynamic_update_node()
    assert node._mission_update_lock.acquire(blocking=False)
    try:
        node._dynamic_r_worker({
            'timestamp_monotonic': time.monotonic(), 'current_seq': 6,
        })
    finally:
        node._mission_update_lock.release()

    assert node._dynamic_update_result == 'UPDATE_FAILED'
    assert any(name == 'r_update_rejected_mission_busy' for name, _ in events)


def test_composite_ard_start_seq_mapping_is_preserved():
    select = FcuInterfaceMavrosNode.select_composite_start_seq

    assert select(0, 5, 9, 5, 8, 14) == (
        1,
        'normalize_seq0_to_first_executable_mission_item',
    )
    assert select(1, 5, 9, 5, 8, 14)[0] == 1
    assert select(2, 5, 9, 5, 8, 14)[0] == 2
    for seq in range(5, 14):
        with pytest.raises(ValueError, match='Late composite insertion'):
            select(seq, 5, 9, 5, 8, 14)


def test_composite_mode_change_does_not_request_mode_or_clear_mission():
    node = make_node_without_ros()
    node.active_task_id = 'target_001'
    node.log_state = 'PLANNING'
    node.mission_type = 'ORIGINAL'
    node.insert_wp_index = 5
    node.resume_wp_index = 8
    node.clear_mission_before_full_push = False
    node.get_logger = lambda: NullLogger()
    original = [
        node.make_nav_waypoint(22.8800 + index * 0.001, 113.4900, 35.0, 15.0)
        for index in range(12)
    ]
    pushed = []
    pull_count = 0

    async def pull_mission_async():
        nonlocal pull_count
        pull_count += 1
        if pull_count == 1:
            return SimpleNamespace(waypoints=original, current_seq=2)
        return SimpleNamespace(waypoints=pushed, current_seq=2)

    async def push_mission_async(waypoints):
        pushed.extend(node.clone_waypoint(wp) for wp in waypoints)

    async def set_current_mission_item_async(_seq):
        return None

    clear_calls = []

    async def clear_mission_async():
        clear_calls.append(True)

    node.pull_mission_async = pull_mission_async
    node.push_mission_async = push_mission_async
    node.set_current_mission_item_async = set_current_mission_item_async
    node.clear_mission_async = clear_mission_async
    node.wait_for_current_seq = lambda _seq, timeout_sec: True
    node.current_mode = lambda: 'GUIDED'
    modes = []
    node.set_mode = lambda mode: modes.append(mode)
    node.publish_mission_summary_event = lambda *_args, **_kwargs: None
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)

    with pytest.raises(
        RuntimeError,
        match='no automatic mode change or LOITER rollback was attempted',
    ):
        asyncio.run(node.handle_goto_global_composite_async(points))

    assert clear_calls == []
    assert modes == []
    assert node.mission_type == 'COMPOSITE'
    assert node.composite_route_points is not None
    assert node.composite_final_seq == node.composite_seq_d


def test_late_composite_request_is_rejected_before_push():
    node = make_node_without_ros()
    node.active_task_id = 'target_001'
    node.log_state = 'PLANNING'
    node.mission_type = 'ORIGINAL'
    node.insert_wp_index = 5
    node.resume_wp_index = 8
    node.clear_mission_before_full_push = False
    node.get_logger = lambda: NullLogger()
    original = [
        node.make_nav_waypoint(22.8800 + index * 0.001, 113.4900, 35.0, 15.0)
        for index in range(12)
    ]

    async def pull_mission_async():
        return SimpleNamespace(waypoints=original, current_seq=5)

    push_calls = []
    set_current_calls = []
    clear_calls = []

    async def push_mission_async(_waypoints):
        push_calls.append(True)

    async def set_current_mission_item_async(seq):
        set_current_calls.append(seq)

    async def clear_mission_async():
        clear_calls.append(True)

    node.pull_mission_async = pull_mission_async
    node.push_mission_async = push_mission_async
    node.set_current_mission_item_async = set_current_mission_item_async
    node.clear_mission_async = clear_mission_async
    node.current_mode = lambda: 'AUTO'
    node.publish_mission_summary_event = lambda *_args, **_kwargs: None
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)

    with pytest.raises(RuntimeError, match='already reached or passed insert_wp_index'):
        asyncio.run(node.handle_goto_global_composite_async(points))

    assert push_calls == []
    assert set_current_calls == []
    assert clear_calls == []
    assert node.mission_type == 'ORIGINAL'


def test_composite_d_event_reports_c_evidence_and_clears_check_state():
    node = make_node_without_ros()
    node.active_task_id = 'target_001'
    node.log_state = 'EXECUTING'
    node.mission_type = 'COMPOSITE'
    node.get_logger = lambda: NullLogger()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    original = [
        node.make_nav_waypoint(22.8800 + index * 0.001, 113.4900, 35.0, 15.0)
        for index in range(12)
    ]
    mission, _ = node.build_composite_mission(points, original, 5, 8)
    node.composite_route_points = {**points}
    node.composite_ab_check_result = {'passed': True}
    node.composite_c_confirmed = True
    node.composite_c_confirmation = 'gps_radius'
    node.composite_c_min_distance_m = 2.5
    node.composite_final_seq = node.composite_seq_d
    node.current_waypoints = SimpleNamespace(
        waypoints=mission,
        current_seq=node.composite_seq_d,
    )
    published = []
    node.composite_complete_publisher = SimpleNamespace(
        publish=lambda message: published.append(message.data)
    )

    node.last_reached_seq = node.composite_seq_d - 1
    node.maybe_report_composite_mission_complete()
    assert published == []

    node.last_reached_seq = node.composite_seq_d
    node.maybe_report_composite_mission_complete()
    assert len(published) == 1
    assert 'c_confirmed=true' in published[0]
    assert f'final_seq={node.composite_seq_d}' in published[0]
    assert node.composite_route_points is None
    assert node.composite_ab_check_result is None


def test_composite_ab_check_and_c_passage_use_current_task_only():
    node = make_node_without_ros()
    node.active_task_id = 'target_001'
    node.log_state = 'EXECUTING'
    node.mission_type = 'COMPOSITE'
    node.get_logger = lambda: NullLogger()
    points = node.compute_abcdr_points(22.8848, 113.4956, 90.0)
    node.composite_route_points = {**points}
    node.composite_seq_a = 5
    node.composite_seq_r = 6
    node.composite_seq_release = None
    node.composite_seq_d = 7
    node.last_reached_seq = node.composite_seq_a
    node.current_waypoints = SimpleNamespace(
        waypoints=[object()] * (node.composite_seq_d + 1),
        current_seq=node.composite_seq_r,
    )
    now = time.time()
    for index, distance_m in enumerate((15.0, 30.0, 45.0)):
        lat, lon = node.destination_point(
            points['A']['lat'], points['A']['lon'], 90.0, distance_m
        )
        node.gps_history.append((now + index * 0.1, lat, lon))
    gps = SimpleNamespace(
        latitude=node.gps_history[-1][1],
        longitude=node.gps_history[-1][2],
        status=SimpleNamespace(status=0),
    )
    node.current_gps = gps
    node.composite_ab_check_state = 'EVALUATING'
    node.finalize_composite_ab_check(force=True)
    assert node.composite_ab_check_result['passed']

    c_gps = SimpleNamespace(
        latitude=points['C']['lat'],
        longitude=points['C']['lon'],
        status=SimpleNamespace(status=0),
    )
    node.current_waypoints.current_seq = node.composite_seq_d
    node.update_composite_trajectory_check(c_gps)
    assert node.composite_c_confirmed
    assert node.composite_c_confirmation == 'gps_radius'
    assert list(node.gps_history) == []
