import time
import unittest

import rclpy
from mavros_msgs.msg import Waypoint, WaypointList
from rclpy.parameter import Parameter
from std_msgs.msg import String

from uav_interfaces.msg import ReconTarget
from uav_vision_bridge.vision_target_bridge_node import VisionTargetBridge


class VisionTargetRetryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = VisionTargetBridge(
            parameter_overrides=[
                Parameter('auto_execute', value=True),
                Parameter('target_retry_interval_sec', value=0.2),
            ]
        )
        state = String()
        state.data = 'STANDBY'
        self.node.mission_state_callback(state)

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def finalized_target(
        target_id='target_001',
        observation_count=20,
        radius_95_m=0.5,
        confidence=1.0,
    ):
        target = ReconTarget()
        target.target_id = target_id
        target.label = '09'
        target.confidence = confidence
        target.latitude = 22.88290413
        target.longitude = 113.48808515
        target.horizontal_radius_95_m = radius_95_m
        target.observation_count = observation_count
        target.valid = True
        target.status = 'finalized'
        return target

    def use_two_candidate_selection(self, timeout_sec=10.0):
        self.node.destroy_node()
        self.node = VisionTargetBridge(
            parameter_overrides=[
                Parameter('auto_execute', value=True),
                Parameter('target_retry_interval_sec', value=0.2),
                Parameter('target_selection_candidate_count', value=2),
                Parameter('candidate_collection_timeout_sec', value=timeout_sec),
                Parameter('target_selection_commit_wp_index', value=4),
                Parameter('target_submission_deadline_wp_index', value=5),
            ]
        )
        state = String()
        state.data = 'STANDBY'
        self.node.mission_state_callback(state)

    def set_mission_seq(self, current_seq, waypoint_count=14):
        waypoints = WaypointList()
        waypoints.current_seq = current_seq
        waypoints.waypoints = [Waypoint() for _ in range(waypoint_count)]
        self.node.mission_waypoints_callback(waypoints)

    def test_retries_while_manager_remains_standby(self):
        self.node.target_callback(self.finalized_target())
        self.assertEqual(1, self.node.target_publish_attempts)

        self.node.target_published_monotonic = time.monotonic() - 1.0
        self.node.drive_automation()

        self.assertEqual(2, self.node.target_publish_attempts)
        self.assertEqual('waiting_for_manager_ack', self.node.automation_phase)

    def test_stops_retrying_after_manager_accepts_target(self):
        self.node.target_callback(self.finalized_target())
        state = String()
        state.data = 'EXECUTING'
        self.node.mission_state_callback(state)
        attempts = self.node.target_publish_attempts
        self.node.target_published_monotonic = time.monotonic() - 1.0

        self.node.drive_automation()

        self.assertEqual(attempts, self.node.target_publish_attempts)
        self.assertEqual('manager_accepted_target', self.node.automation_phase)

    def test_two_candidate_mode_selects_stronger_second_final(self):
        self.use_two_candidate_selection()
        self.set_mission_seq(3)
        weak = self.finalized_target(
            target_id='target_001',
            observation_count=36,
            radius_95_m=1.2,
            confidence=0.92,
        )
        strong = self.finalized_target(
            target_id='target_002',
            observation_count=55,
            radius_95_m=0.4,
            confidence=0.99,
        )

        self.node.target_callback(weak)
        self.assertIsNone(self.node.pending_target)
        self.assertEqual(0, self.node.target_publish_attempts)

        self.node.target_callback(strong)

        self.assertEqual('target_002', self.node.pending_target.target_id)
        self.assertEqual(1, self.node.target_publish_attempts)

    def test_duplicate_final_does_not_fill_two_candidate_quota(self):
        self.use_two_candidate_selection()
        self.set_mission_seq(3)
        target = self.finalized_target(target_id='target_001')

        self.node.target_callback(target)
        self.node.target_callback(target)

        self.assertEqual(1, len(self.node.candidate_targets))
        self.assertIsNone(self.node.pending_target)
        self.assertEqual(0, self.node.target_publish_attempts)

    def test_candidate_collection_timeout_uses_best_available_final(self):
        self.use_two_candidate_selection(timeout_sec=10.0)
        self.set_mission_seq(3)
        target = self.finalized_target(target_id='target_001')
        self.node.target_callback(target)
        self.node.candidate_collection_started_monotonic = time.monotonic() - 11.0

        self.node.drive_automation()

        self.assertEqual('target_001', self.node.pending_target.target_id)
        self.assertEqual(1, self.node.target_publish_attempts)

    def test_equal_frame_count_prefers_lower_radius_95(self):
        self.use_two_candidate_selection()
        self.set_mission_seq(3)
        first = self.finalized_target(
            target_id='target_001',
            observation_count=40,
            radius_95_m=1.0,
            confidence=1.0,
        )
        second = self.finalized_target(
            target_id='target_002',
            observation_count=40,
            radius_95_m=0.3,
            confidence=0.9,
        )

        self.node.target_callback(first)
        self.node.target_callback(second)

        self.assertEqual('target_002', self.node.pending_target.target_id)

    def test_two_candidate_mode_keeps_stronger_first_final(self):
        self.use_two_candidate_selection()
        self.set_mission_seq(3)
        strong = self.finalized_target(
            target_id='target_001',
            observation_count=60,
            radius_95_m=0.4,
            confidence=0.98,
        )
        weak = self.finalized_target(
            target_id='target_002',
            observation_count=20,
            radius_95_m=0.2,
            confidence=1.0,
        )

        self.node.target_callback(strong)
        self.node.target_callback(weak)

        self.assertEqual('target_001', self.node.pending_target.target_id)

    def test_waypoint_four_commits_single_existing_candidate(self):
        self.use_two_candidate_selection()
        self.set_mission_seq(3)
        target = self.finalized_target(target_id='target_001')
        self.node.target_callback(target)

        self.assertIsNone(self.node.pending_target)
        self.assertEqual(0, self.node.target_publish_attempts)

        self.set_mission_seq(4)

        self.assertEqual('target_001', self.node.pending_target.target_id)
        self.assertEqual(1, self.node.target_publish_attempts)

    def test_final_arriving_at_waypoint_four_is_committed_immediately(self):
        self.use_two_candidate_selection()
        self.set_mission_seq(4)

        self.node.target_callback(self.finalized_target(target_id='target_001'))

        self.assertEqual('target_001', self.node.pending_target.target_id)
        self.assertEqual(1, self.node.target_publish_attempts)

    def test_waypoint_three_still_waits_for_second_candidate(self):
        self.use_two_candidate_selection()
        self.set_mission_seq(3)

        self.node.target_callback(self.finalized_target(target_id='target_001'))

        self.assertIsNone(self.node.pending_target)
        self.assertEqual(0, self.node.target_publish_attempts)

    def test_waypoint_five_stops_retries_after_window_closes(self):
        self.use_two_candidate_selection()
        self.set_mission_seq(4)
        self.node.target_callback(self.finalized_target(target_id='target_001'))
        self.assertEqual(1, self.node.target_publish_attempts)

        self.set_mission_seq(5)
        attempts = self.node.target_publish_attempts
        self.node.target_published_monotonic = time.monotonic() - 1.0
        self.node.drive_automation()

        self.assertEqual(attempts, self.node.target_publish_attempts)
        self.assertEqual('failed', self.node.automation_phase)


if __name__ == '__main__':
    unittest.main()
