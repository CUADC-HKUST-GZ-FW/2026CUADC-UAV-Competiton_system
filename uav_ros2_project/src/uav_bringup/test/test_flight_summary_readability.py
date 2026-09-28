"""Regression tests for the human-readable mission summary contract.

These tests intentionally protect readability, not flight-control behaviour.
Detailed telemetry must stay in all_nodes.log / aburcd_update_metrics.jsonl.
"""

from uav_bringup.flight_summary_logger_node import FlightSummaryLoggerNode


def _formatter_only_node():
    """Build a formatter-only instance without starting ROS subscriptions."""
    node = FlightSummaryLoggerNode.__new__(FlightSummaryLoggerNode)
    node.summary_path = '/tmp/flight_summary.log'
    node.target_topic = '/vision/target_command'
    node.safe_r_point = None
    node.dynamic_r_point = None
    node.r_point = None
    node.release_snapshot = None
    node._aburcd_metrics = {}
    node.dynamic_r_enabled = True
    return node


def test_unknown_event_is_not_dumped_into_human_summary():
    """New telemetry events must not silently turn summary into a debug log."""
    node = _formatter_only_node()
    text = node._format_human_event({
        'time': '2026-09-21T12:00:00+08:00',
        'event': 'future_internal_debug_event',
        'task_id': 'target_001',
        'large_internal_value': 123.456,
        'another_internal_value': 'noise',
    })
    assert text is None


def test_target_summary_stays_compact():
    """Target entry keeps flight-review facts but drops ROS graph detail."""
    node = _formatter_only_node()
    text = node._format_human_event({
        'time': '2026-09-21T12:00:00+08:00',
        'event': 'vision_target_received',
        'task_id': 'target_001',
        'latitude': 22.8829116,
        'longitude': 113.4880722,
        'heading_deg': 0.0,
        'source_topic': '/vision/target_command',
        'source_node': '/sitl_target_gate_node',
        'source_resolution': 'graph_unique',
        'publisher_count': 1,
    })
    assert 'lat/lon:' in text
    assert 'source: /sitl_target_gate_node' in text
    assert 'source topic:' not in text
    assert 'source resolution:' not in text
    assert 'publisher count:' not in text


def test_dynamic_intermediate_events_remain_suppressed():
    """High-rate prediction/transaction stages belong in detailed logs only."""
    suppressed = FlightSummaryLoggerNode._SUMMARY_SUPPRESSED_EVENTS
    for event in (
        'ab_prediction_input',
        'r_prediction_iteration',
        'second_full_push_start',
        'second_full_push_done',
        'mission_progress_check',
        'mission_progress_safe',
    ):
        assert event in suppressed
