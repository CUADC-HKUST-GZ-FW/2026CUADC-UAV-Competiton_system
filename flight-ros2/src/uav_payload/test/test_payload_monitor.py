from pathlib import Path
from types import SimpleNamespace
import unittest

from uav_payload.payload_monitor import (
    MAV_CMD_DO_SET_SERVO,
    PayloadMonitor,
    PayloadMonitorConfig,
    PayloadMonitorState,
)


def waypoint(command=16, channel=0, pwm=0, lat=0.0, lon=0.0):
    return SimpleNamespace(
        command=command,
        param1=float(channel),
        param2=float(pwm),
        x_lat=float(lat),
        y_long=float(lon),
    )


def mission(release_seq=4, channel=7, pwm=1900):
    result = [waypoint(lat=22.0 + index * 0.001) for index in range(7)]
    result[release_seq] = waypoint(MAV_CMD_DO_SET_SERVO, channel, pwm)
    return result


class PayloadMonitorTest(unittest.TestCase):
    def make_monitor(self, **overrides):
        values = {
            'servo_channel': 7,
            'release_pwm': 1900,
            'pwm_tolerance_us': 20,
            'required_consecutive_samples': 3,
            'telemetry_stale_timeout_s': 1.0,
            'execution_timeout_s': 5.0,
            'warning_throttle_s': 5.0,
        }
        values.update(overrides)
        monitor = PayloadMonitor(PayloadMonitorConfig(**values))
        monitor.set_task_id('target_001', 0.0)
        return monitor

    def test_matching_command_is_detected_at_dynamic_sequence(self):
        monitor = self.make_monitor()
        events = monitor.observe_mission(mission(release_seq=4), 0, 1.0)
        self.assertEqual(4, monitor.release_command_seq)
        self.assertEqual(['command_uploaded'], [event.key for event in events])

    def test_no_release_command_is_not_reported_as_uploaded(self):
        monitor = self.make_monitor()
        events = monitor.observe_mission([waypoint(), waypoint()], 0, 1.0)
        self.assertFalse(monitor.release_command_seen)
        self.assertNotIn('command_uploaded', [event.key for event in events])

    def test_wrong_channel_does_not_match(self):
        monitor = self.make_monitor()
        monitor.observe_mission(mission(channel=8), 0, 1.0)
        self.assertIsNone(monitor.release_command_seq)

    def test_wrong_pwm_does_not_match(self):
        monitor = self.make_monitor()
        monitor.observe_mission(mission(pwm=1800), 0, 1.0)
        self.assertIsNone(monitor.release_command_seq)

    def test_multiple_matching_commands_are_invalid(self):
        monitor = self.make_monitor()
        waypoints = mission()
        waypoints[5] = waypoint(MAV_CMD_DO_SET_SERVO, 7, 1900)
        events = monitor.observe_mission(waypoints, 0, 1.0)
        self.assertEqual(PayloadMonitorState.INVALID, monitor.state)
        self.assertEqual('ambiguous_release_commands', events[0].key)

    def test_waypoint_reached_marks_command_reached(self):
        monitor = self.make_monitor()
        monitor.observe_mission(mission(), 2, 1.0)
        events = monitor.observe_waypoint_reached(4, 2.0)
        self.assertTrue(monitor.release_command_reached)
        self.assertIn('command_reached', [event.key for event in events])

    def test_current_sequence_passage_marks_reached(self):
        monitor = self.make_monitor()
        monitor.observe_mission(mission(), 2, 1.0)
        monitor.observe_current_seq(4, 2.0)
        events = monitor.observe_current_seq(5, 2.1)
        self.assertEqual('command_reached', events[0].key)

    def test_servo_channel_seven_uses_array_index_six(self):
        monitor = self.make_monitor(required_consecutive_samples=1)
        monitor.observe_mission(mission(), 4, 1.0)
        monitor.observe_waypoint_reached(4, 1.1)
        channels = [1000] * 8
        channels[6] = 1900
        monitor.observe_rc_out(channels, 1.2)
        self.assertTrue(monitor.release_pwm_confirmed)

    def test_short_channel_array_warns_without_crashing(self):
        monitor = self.make_monitor()
        events = monitor.observe_rc_out([1500] * 6, 1.0)
        self.assertEqual('channel_unavailable', events[0].key)

    def test_one_pwm_sample_is_not_confirmation(self):
        monitor = self.make_monitor()
        monitor.observe_mission(mission(), 4, 1.0)
        monitor.observe_waypoint_reached(4, 1.1)
        monitor.observe_rc_out([1500] * 6 + [1900], 1.2)
        self.assertFalse(monitor.release_pwm_confirmed)

    def test_three_consecutive_pwm_samples_confirm(self):
        monitor = self.make_monitor()
        monitor.observe_mission(mission(), 4, 1.0)
        monitor.observe_waypoint_reached(4, 1.1)
        events = []
        for now in (1.2, 1.3, 1.4):
            events.extend(monitor.observe_rc_out([1500] * 6 + [1900], now))
        self.assertTrue(monitor.release_pwm_confirmed)
        self.assertIn('pwm_confirmed', [event.key for event in events])

    def test_mismatch_resets_consecutive_count(self):
        monitor = self.make_monitor()
        monitor.observe_mission(mission(), 4, 1.0)
        monitor.observe_waypoint_reached(4, 1.1)
        monitor.observe_rc_out([1500] * 6 + [1900], 1.2)
        monitor.observe_rc_out([1500] * 7, 1.3)
        monitor.observe_rc_out([1500] * 6 + [1900], 1.4)
        self.assertEqual(1, monitor.consecutive_pwm_matches)

    def test_pwm_confirmation_is_latched_across_task_change(self):
        monitor = self.make_monitor(required_consecutive_samples=1)
        monitor.observe_mission(mission(), 4, 1.0)
        monitor.observe_waypoint_reached(4, 1.1)
        monitor.observe_rc_out([1500] * 6 + [1900], 1.2)
        events = monitor.set_task_id('none', 2.0)
        self.assertTrue(monitor.release_pwm_confirmed)
        self.assertTrue(monitor.release_confirmed_latched)
        self.assertEqual(PayloadMonitorState.PWM_CONFIRMED, monitor.state)
        self.assertEqual(4, monitor.release_command_seq)
        self.assertEqual(['release_already_confirmed'], [event.key for event in events])

    def test_core_events_are_emitted_once(self):
        monitor = self.make_monitor()
        first = monitor.observe_mission(mission(), 0, 1.0)
        second = monitor.observe_mission(mission(), 0, 1.1)
        self.assertEqual(1, len(first))
        self.assertEqual([], second)

    def test_stale_telemetry_warning_is_throttled(self):
        monitor = self.make_monitor()
        monitor.observe_mission(mission(), 4, 1.0)
        monitor.observe_waypoint_reached(4, 1.1)
        first = monitor.tick(2.2)
        second = monitor.tick(2.3)
        self.assertEqual('rc_telemetry_stale', first[0].key)
        self.assertEqual([], second)

    def test_execution_timeout_enters_timeout_state(self):
        monitor = self.make_monitor(execution_timeout_s=2.0)
        monitor.observe_mission(mission(), 3, 1.0)
        monitor.observe_waypoint_reached(3, 1.0)
        events = monitor.tick(3.1)
        self.assertEqual(PayloadMonitorState.TIMEOUT, monitor.state)
        self.assertTrue(events[0].key.startswith('timeout:'))

    def test_execution_timeout_does_not_start_when_release_is_far_away(self):
        monitor = self.make_monitor(execution_timeout_s=2.0)
        monitor.observe_mission(mission(release_seq=6), 1, 1.0)

        events = monitor.tick(100.0)

        self.assertEqual([], events)
        self.assertFalse(monitor.execution_window_armed)
        self.assertIsNone(monitor.execution_window_started_at)
        self.assertEqual(
            PayloadMonitorState.WAITING_FOR_COMMAND_REACHED,
            monitor.state,
        )

    def test_current_seq_does_not_arm_execution_window(self):
        monitor = self.make_monitor(execution_timeout_s=2.0)
        monitor.observe_mission(mission(release_seq=6), 4, 1.0)

        events = monitor.observe_current_seq(5, 10.0)

        self.assertEqual([], events)
        self.assertFalse(monitor.execution_window_armed)
        self.assertIsNone(monitor.execution_window_started_at)

    def test_reached_event_arms_dynamically_one_seq_before_release(self):
        monitor = self.make_monitor(execution_timeout_s=2.0)
        monitor.observe_mission(mission(release_seq=6), 5, 1.0)

        events = monitor.observe_waypoint_reached(5, 10.0)

        self.assertTrue(monitor.execution_window_armed)
        self.assertEqual(10.0, monitor.execution_window_started_at)
        self.assertEqual(['execution_window_armed'], [event.key for event in events])
        self.assertIn('release_seq=6 reached_seq=5', events[0].message)
        self.assertEqual([], monitor.observe_waypoint_reached(5, 10.1))

    def test_timeout_can_recover_from_late_seq_and_pwm_evidence(self):
        monitor = self.make_monitor(
            execution_timeout_s=2.0,
            required_consecutive_samples=1,
        )
        monitor.observe_mission(mission(release_seq=4), 3, 1.0)
        monitor.observe_waypoint_reached(3, 1.0)
        monitor.tick(3.1)
        self.assertEqual(PayloadMonitorState.TIMEOUT, monitor.state)

        monitor.observe_current_seq(5, 3.2)
        events = monitor.observe_rc_out([1500] * 6 + [1900], 3.3)

        self.assertEqual(
            ['command_reached', 'pwm_confirmed'],
            [event.key for event in events],
        )
        self.assertEqual(PayloadMonitorState.PWM_CONFIRMED, monitor.state)
        self.assertTrue(monitor.release_confirmed_latched)

    def test_latched_monitor_does_not_rescan_retained_mission(self):
        monitor = self.make_monitor(required_consecutive_samples=1)
        retained_mission = mission(release_seq=4)
        monitor.observe_mission(retained_mission, 4, 1.0)
        monitor.observe_waypoint_reached(4, 1.1)
        monitor.observe_rc_out([1500] * 6 + [1900], 1.2)
        monitor.set_task_id('none', 2.0)

        events = monitor.observe_mission(retained_mission, 6, 2.1)

        self.assertEqual([], events)
        self.assertEqual(PayloadMonitorState.PWM_CONFIRMED, monitor.state)
        self.assertTrue(monitor.release_confirmed_latched)
        self.assertEqual(4, monitor.release_command_seq)

    def test_pwm_is_not_accepted_before_command_reached(self):
        monitor = self.make_monitor(required_consecutive_samples=1)
        monitor.observe_mission(mission(), 0, 1.0)
        monitor.observe_rc_out([1500] * 6 + [1900], 1.1)
        self.assertFalse(monitor.release_pwm_confirmed)

    def test_invalid_configuration_fails_closed(self):
        with self.assertRaises(ValueError):
            PayloadMonitor(PayloadMonitorConfig(servo_channel=0))

    def test_ros_monitor_node_contains_no_control_clients(self):
        node_source = (
            Path(__file__).parents[1]
            / 'uav_payload'
            / 'payload_monitor_node.py'
        ).read_text(encoding='utf-8')
        self.assertNotIn('create_client(', node_source)
        self.assertNotIn('mission/push', node_source)
        self.assertNotIn('cmd/command', node_source)


if __name__ == '__main__':
    unittest.main()
