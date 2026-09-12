import struct
import time
import unittest
from unittest.mock import Mock

import rclpy
from mavros_msgs.msg import Mavlink, State
from std_msgs.msg import String
from std_srvs.srv import Trigger

from uav_interfaces.msg import TargetCommand
from uav_mission_manager.mission_manager_node import MissionManagerNode, MissionState


class MissionSafetyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = MissionManagerNode()

    def tearDown(self):
        self.node.destroy_node()

    def make_standby_healthy(self):
        now = time.monotonic()
        self.node.fcu_connected = True
        self.node.fcu_armed = True
        self.node.fcu_mode = 'AUTO'
        self.node.fcu_system_status = 3
        self.node.actual_system_id = self.node.expected_system_id
        self.node.actual_component_id = self.node.expected_component_id
        self.node.last_fcu_state_time = now
        self.node.last_heartbeat_time = now
        self.node.heartbeat_count = self.node.heartbeat_required_count
        self.node.heartbeat_sequence_start = (
            now - self.node.heartbeat_stable_duration_sec
        )
        self.node.last_vehicle_info_time = now
        self.node.last_position_time = now
        self.node.position_healthy = True
        self.node.last_gps_time = now
        self.node.gps_healthy = True
        self.node.last_ekf_time = now
        self.node.ekf_healthy = True
        self.node.last_sensor_time = now
        self.node.sensor_health = True
        self.node.current_mission_seq = 2
        self.node.current_mission_count = 15
        self.node.last_mission_waypoints_time = now
        self.node.goto_client = Mock()
        self.node.goto_client.service_is_ready.return_value = True
        self.node.transition_to(MissionState.STANDBY, 'test_link_stable')

    def test_boot_never_restores_authority_or_target(self):
        self.assertEqual(MissionState.WAIT_FCU, self.node.state)
        self.assertFalse(self.node.execution_allowed)
        self.assertFalse(self.node.attack_executed)
        self.assertFalse(self.node.target_valid)
        self.assertIsNone(self.node.active_target)

    def test_valid_target_auto_executes_once(self):
        self.make_standby_healthy()
        started = []
        self.node.goto_target_process_point = (
            lambda generation: started.append(generation)
        )
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.EXECUTING, self.node.state)
        self.assertTrue(self.node.execution_allowed)
        self.assertTrue(self.node.attack_executed)
        self.assertTrue(self.node.target_valid)
        self.assertEqual(1, len(started))

    def test_target_before_insert_wp_index_auto_executes(self):
        self.make_standby_healthy()
        self.node.current_mission_seq = self.node.insert_wp_index - 1
        started = []
        self.node.goto_target_process_point = (
            lambda generation: started.append(generation)
        )
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.EXECUTING, self.node.state)
        self.assertTrue(self.node.target_valid)
        self.assertEqual(1, len(started))

    def test_target_at_insert_wp_index_is_rejected(self):
        self.make_standby_healthy()
        self.node.current_mission_seq = self.node.insert_wp_index
        started = []
        self.node.goto_target_process_point = (
            lambda generation: started.append(generation)
        )
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertFalse(self.node.target_valid)
        self.assertIsNone(self.node.active_target)
        self.assertEqual([], started)

    def test_target_after_insert_wp_index_is_rejected(self):
        self.make_standby_healthy()
        self.node.current_mission_seq = self.node.insert_wp_index + 1
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertFalse(self.node.target_valid)
        self.assertIsNone(self.node.active_target)

    def test_target_is_rejected_when_mission_current_seq_unknown(self):
        self.make_standby_healthy()
        self.node.current_mission_seq = None
        self.node.current_mission_count = 0
        self.node.last_mission_waypoints_time = None
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertFalse(self.node.target_valid)
        self.assertIn(
            'insert_window_mission_current_seq_unknown',
            self.node._attack_start_blockers_locked(time.monotonic()),
        )

    def test_event_driven_waypoint_cache_does_not_expire_by_age(self):
        self.make_standby_healthy()
        self.node.last_mission_waypoints_time = time.monotonic() - 60.0
        started = []
        self.node.goto_target_process_point = (
            lambda generation: started.append(generation)
        )
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.EXECUTING, self.node.state)
        self.assertTrue(self.node.target_valid)
        self.assertEqual(1, len(started))

    def test_disconnect_invalidates_cached_waypoint_state(self):
        self.make_standby_healthy()
        state = State()
        state.connected = False

        self.node.fcu_state_callback(state)

        self.assertIsNone(self.node.current_mission_seq)
        self.assertEqual(0, self.node.current_mission_count)
        self.assertIsNone(self.node.last_mission_waypoints_time)
        self.assertIn(
            'insert_window_mission_current_seq_unknown',
            self.node._attack_start_blockers_locked(time.monotonic()),
        )

    def test_insert_window_gate_can_be_disabled(self):
        self.make_standby_healthy()
        self.node.current_mission_seq = self.node.insert_wp_index + 1
        self.node.reject_target_at_or_after_insert_wp = False
        started = []
        self.node.goto_target_process_point = (
            lambda generation: started.append(generation)
        )
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.EXECUTING, self.node.state)
        self.assertTrue(self.node.target_valid)
        self.assertEqual(1, len(started))

    def test_wait_fcu_transitions_directly_to_standby(self):
        self.make_standby_healthy()

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertNotIn('CHECK_SYSTEM', MissionState.__members__)

    def test_unarmed_target_is_rejected_without_latching(self):
        self.make_standby_healthy()
        self.node.fcu_armed = False
        started = []
        self.node.goto_target_process_point = (
            lambda generation: started.append(generation)
        )
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertFalse(self.node.attack_executed)
        self.assertFalse(self.node.execution_allowed)
        self.assertFalse(self.node.busy)
        self.assertFalse(self.node.target_valid)
        self.assertIsNone(self.node.active_target)
        self.assertEqual([], started)
        self.assertIn(
            'aircraft_not_armed',
            self.node._attack_start_blockers_locked(time.monotonic()),
        )

    def test_non_auto_target_is_rejected_without_latching(self):
        self.make_standby_healthy()
        self.node.fcu_mode = 'FBWA'
        started = []
        self.node.goto_target_process_point = (
            lambda generation: started.append(generation)
        )
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0

        self.node.target_callback(target)

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertFalse(self.node.attack_executed)
        self.assertFalse(self.node.target_valid)
        self.assertIsNone(self.node.active_target)
        self.assertEqual([], started)
        self.assertIn(
            'flight_mode_not_auto',
            self.node._attack_start_blockers_locked(time.monotonic()),
        )

    def test_attack_gate_separates_stale_from_bad_navigation(self):
        self.make_standby_healthy()
        now = time.monotonic()
        self.node.last_position_time = now
        self.node.position_healthy = False
        self.node.last_gps_time = now
        self.node.gps_healthy = False
        self.node.last_ekf_time = now
        self.node.ekf_healthy = False

        bad_blockers = self.node._attack_start_blockers_locked(now)

        self.assertIn('position_invalid', bad_blockers)
        self.assertIn('gps_health_bad', bad_blockers)
        self.assertIn('ekf_health_bad', bad_blockers)
        self.assertNotIn('position_data_stale', bad_blockers)
        self.assertNotIn('gps_data_stale', bad_blockers)
        self.assertNotIn('ekf_data_stale', bad_blockers)

        self.node.last_position_time = None
        self.node.last_gps_time = None
        self.node.last_ekf_time = None
        stale_blockers = self.node._attack_start_blockers_locked(now)

        self.assertIn('position_data_stale', stale_blockers)
        self.assertIn('gps_data_stale', stale_blockers)
        self.assertIn('ekf_data_stale', stale_blockers)
        self.assertNotIn('position_invalid', stale_blockers)
        self.assertNotIn('gps_health_bad', stale_blockers)
        self.assertNotIn('ekf_health_bad', stale_blockers)

    def test_standby_gps_stale_warns_and_recovers_attack_readiness(self):
        self.make_standby_healthy()
        self.node.last_gps_time = (
            time.monotonic() - self.node.gps_timeout_sec - 0.1
        )

        self.node.health_timer_callback()
        stale_snapshot = self.node._status_snapshot_locked()

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertFalse(stale_snapshot['ready_for_attack'])
        self.assertIn('gps_data_stale', stale_snapshot['runtime_faults'])
        self.assertEqual('WARNING', stale_snapshot['runtime_health_level'])

        self.node.last_gps_time = time.monotonic()
        self.node.gps_healthy = True
        self.node.health_timer_callback()
        recovered_snapshot = self.node._status_snapshot_locked()

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertTrue(recovered_snapshot['ready_for_attack'])
        self.assertNotIn('gps_data_stale', recovered_snapshot['runtime_faults'])

    def test_standby_link_stale_returns_to_wait_fcu(self):
        self.make_standby_healthy()
        self.node.last_fcu_state_time = (
            time.monotonic() - self.node.fcu_state_timeout_sec - 0.1
        )

        self.node.health_timer_callback()

        self.assertEqual(MissionState.WAIT_FCU, self.node.state)
        self.assertFalse(self.node.execution_allowed)

    def test_status_snapshot_exposes_attack_and_runtime_diagnostics(self):
        self.make_standby_healthy()

        snapshot = self.node._status_snapshot_locked()

        self.assertTrue(snapshot['ready_for_attack'])
        self.assertEqual([], snapshot['attack_start_blockers'])
        self.assertEqual('OK', snapshot['runtime_health_level'])
        self.assertIn('armed', snapshot)
        self.assertIn('mode', snapshot)
        self.assertIn('runtime_fault_durations_sec', snapshot)
        self.assertEqual(2, snapshot['current_mission_seq'])
        self.assertEqual(self.node.insert_wp_index, snapshot['insert_wp_index'])
        self.assertTrue(snapshot['target_insert_window_open'])

    def test_executing_telemetry_stale_does_not_enter_safe(self):
        self.make_standby_healthy()
        self.node.active_target = {'id': 'target_test'}
        self.node.target_valid = True
        self.node.attack_executed = True
        self.node.transition_to(MissionState.EXECUTING, 'test_start')
        stale_time = time.monotonic() - max(
            self.node.position_timeout_sec,
            self.node.gps_timeout_sec,
            self.node.ekf_timeout_sec,
        ) - 0.1
        self.node.last_position_time = stale_time
        self.node.last_gps_time = stale_time
        self.node.last_ekf_time = stale_time

        self.node.health_timer_callback()

        self.assertEqual(MissionState.EXECUTING, self.node.state)
        self.assertEqual('WARNING', self.node.runtime_health_level.value)
        self.assertIn('position_data_stale', self.node.runtime_faults)
        self.assertIn('gps_data_stale', self.node.runtime_faults)
        self.assertIn('ekf_data_stale', self.node.runtime_faults)

    def test_executing_persistent_stale_becomes_degraded_not_safe(self):
        self.make_standby_healthy()
        self.node.active_target = {'id': 'target_test'}
        self.node.target_valid = True
        self.node.attack_executed = True
        self.node.transition_to(MissionState.EXECUTING, 'test_start')
        self.node.last_gps_time = None
        self.node.runtime_degraded_after_sec = 0.01
        self.node.health_timer_callback()
        self.node.runtime_fault_since['gps_data_stale'] -= 0.02

        self.node.health_timer_callback()

        self.assertEqual(MissionState.EXECUTING, self.node.state)
        self.assertEqual('DEGRADED', self.node.runtime_health_level.value)

    def test_executing_fresh_serious_status_enters_safe(self):
        self.make_standby_healthy()
        self.node.active_target = {'id': 'target_test'}
        self.node.target_valid = True
        self.node.attack_executed = True
        self.node.transition_to(MissionState.EXECUTING, 'test_start')
        self.node.fcu_system_status = next(
            iter(self.node.serious_system_statuses)
        )
        self.node.last_fcu_state_time = time.monotonic()

        self.node.health_timer_callback()

        self.assertEqual(MissionState.SAFE, self.node.state)
        self.assertFalse(self.node.execution_allowed)
        self.assertTrue(self.node.attack_executed)

        self.node.health_timer_callback()
        self.assertEqual(MissionState.SAFE, self.node.state)

        self.node.fcu_system_status = 3
        self.node.last_fcu_state_time = time.monotonic()
        self.node.health_timer_callback()
        self.assertEqual(MissionState.WAIT_FCU, self.node.state)

    def test_safe_clears_authority_and_target(self):
        self.make_standby_healthy()
        self.node.active_target = {'id': 'target_test'}
        self.node.target_valid = True
        self.node.attack_executed = True
        self.node.transition_to(MissionState.EXECUTING, 'test_start')

        self.node.enter_safe('test_fault')

        self.assertEqual(MissionState.SAFE, self.node.state)
        self.assertFalse(self.node.execution_allowed)
        self.assertTrue(self.node.attack_executed)
        self.assertFalse(self.node.target_valid)
        self.assertIsNone(self.node.active_target)
        self.assertFalse(hasattr(self.node, 'historical_target'))

    def test_second_target_is_rejected_after_completion(self):
        self.make_standby_healthy()
        started = []
        self.node.goto_target_process_point = (
            lambda generation: started.append(generation)
        )
        target = TargetCommand()
        target.latitude = 22.8848
        target.longitude = 113.4956
        target.heading_deg = 90.0
        self.node.target_callback(target)
        self.node.complete_mission('test_complete')

        second_target = TargetCommand()
        second_target.latitude = 22.8850
        second_target.longitude = 113.4958
        second_target.heading_deg = 100.0
        self.node.target_callback(second_target)

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertTrue(self.node.attack_executed)
        self.assertFalse(self.node.target_valid)
        self.assertEqual(1, len(started))

    def test_disable_during_execution_enters_safe(self):
        self.make_standby_healthy()
        self.node.active_target = {'id': 'target_test'}
        self.node.target_valid = True
        self.node.attack_executed = True
        self.node.transition_to(MissionState.EXECUTING, 'test_start')

        response = self.node.disable_callback(Trigger.Request(), Trigger.Response())

        self.assertTrue(response.success)
        self.assertEqual(MissionState.SAFE, self.node.state)
        self.assertFalse(self.node.execution_allowed)
        self.assertTrue(self.node.attack_executed)

    def test_composite_completion_at_d_finishes_target_not_full_route(self):
        self.make_standby_healthy()
        self.node.active_target = {'id': 'target_001'}
        self.node.target_valid = True
        self.node.attack_executed = True
        self.node.transition_to(MissionState.EXECUTING, 'test_start')
        event = String()
        event.data = (
            'task_id=target_001 mission_type=COMPOSITE '
            'final_seq=9 total_count=14 last_reached_seq=9 '
            'ab_track_passed=true c_confirmed=true '
            'c_evidence=gps_radius c_min_distance_m=2.50'
        )

        self.node.composite_mission_complete_callback(event)

        self.assertEqual(MissionState.STANDBY, self.node.state)
        self.assertFalse(self.node.execution_allowed)
        self.assertFalse(self.node.busy)
        self.assertTrue(self.node.attack_executed)

    def test_composite_completion_rejects_wrong_task(self):
        self.make_standby_healthy()
        self.node.active_target = {'id': 'target_001'}
        self.node.target_valid = True
        self.node.attack_executed = True
        self.node.transition_to(MissionState.EXECUTING, 'test_start')
        event = String()
        event.data = (
            'task_id=target_999 mission_type=COMPOSITE '
            'final_seq=9 total_count=14 last_reached_seq=9'
        )

        self.node.composite_mission_complete_callback(event)

        self.assertEqual(MissionState.EXECUTING, self.node.state)

    @staticmethod
    def make_ekf_report(flags, system_id=1, message_id=193):
        msg = Mavlink()
        msg.msgid = message_id
        msg.sysid = system_id
        msg.compid = 1
        msg.len = 22
        payload = struct.pack('<5fH', 0.1, 0.2, 0.3, 0.4, 0.5, flags)
        payload += bytes(24 - len(payload))
        msg.payload64 = list(struct.unpack('<3Q', payload))
        return msg

    def test_ardupilot_ekf_report_marks_healthy(self):
        flags = 0x01 | 0x10 | 0x20
        self.node.mavlink_ekf_callback(self.make_ekf_report(flags))

        self.assertTrue(self.node.ekf_healthy)
        self.assertIsNotNone(self.node.last_ekf_time)
        self.assertEqual(
            'ardupilot_ekf_status_report_193', self.node.ekf_source
        )

    def test_ardupilot_ekf_report_rejects_uninitialized(self):
        flags = 0x01 | 0x10 | 0x20 | 0x400
        self.node.mavlink_ekf_callback(self.make_ekf_report(flags))

        self.assertFalse(self.node.ekf_healthy)

    def test_ardupilot_ekf_report_rejects_missing_vertical_position(self):
        flags = 0x01 | 0x10
        self.node.mavlink_ekf_callback(self.make_ekf_report(flags))

        self.assertFalse(self.node.ekf_healthy)

    def test_ardupilot_ekf_report_ignores_wrong_source(self):
        flags = 0x01 | 0x10 | 0x20
        self.node.mavlink_ekf_callback(
            self.make_ekf_report(flags, system_id=2)
        )
        self.node.mavlink_ekf_callback(
            self.make_ekf_report(flags, message_id=230)
        )

        self.assertIsNone(self.node.last_ekf_time)
        self.assertFalse(self.node.ekf_healthy)


if __name__ == '__main__':
    unittest.main()
