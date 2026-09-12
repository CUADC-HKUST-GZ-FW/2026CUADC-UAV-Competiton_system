import time
import unittest

import rclpy
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
    def confirmed_target():
        target = ReconTarget()
        target.target_id = 'target_001'
        target.label = '09'
        target.confidence = 1.0
        target.latitude = 22.88290413
        target.longitude = 113.48808515
        target.valid = True
        target.status = 'confirmed'
        return target

    def test_retries_while_manager_remains_standby(self):
        self.node.target_callback(self.confirmed_target())
        self.assertEqual(1, self.node.target_publish_attempts)

        self.node.target_published_monotonic = time.monotonic() - 1.0
        self.node.drive_automation()

        self.assertEqual(2, self.node.target_publish_attempts)
        self.assertEqual('waiting_for_manager_ack', self.node.automation_phase)

    def test_stops_retrying_after_manager_accepts_target(self):
        self.node.target_callback(self.confirmed_target())
        state = String()
        state.data = 'EXECUTING'
        self.node.mission_state_callback(state)
        attempts = self.node.target_publish_attempts
        self.node.target_published_monotonic = time.monotonic() - 1.0

        self.node.drive_automation()

        self.assertEqual(attempts, self.node.target_publish_attempts)
        self.assertEqual('manager_accepted_target', self.node.automation_phase)


if __name__ == '__main__':
    unittest.main()
