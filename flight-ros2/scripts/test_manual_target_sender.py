"""Pure readiness tests and opt-in ROS tests using dummy nodes, without FCU."""

import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

from manual_target_sender import create_sender, pending_reason


def ready_status():
    return {
        'state': 'STANDBY', 'attack_executed': False, 'target_valid': False,
        'ready_for_attack': True, 'attack_start_blockers': [],
    }


class ReadinessTest(unittest.TestCase):
    def test_ready_snapshot(self):
        self.assertIsNone(pending_reason(ready_status()))

    def test_missing_or_malformed_status_never_authorizes(self):
        for value in (None, [], True, 'ready', {}):
            with self.subTest(value=value):
                self.assertIsNotNone(pending_reason(value))
        for key in ready_status():
            status = ready_status()
            del status[key]
            with self.subTest(missing=key):
                self.assertIsNotNone(pending_reason(status))

    def test_blockers_states_and_previous_execution(self):
        for field, value in (
            ('state', 'WAIT_FCU'), ('state', 'SAFE'), ('state', 'EXECUTING'),
            ('ready_for_attack', False), ('ready_for_attack', 'true'),
            ('ready_for_attack', 1), ('attack_executed', True),
            ('target_valid', True),
            ('attack_start_blockers', ['aircraft_not_armed']),
            ('attack_start_blockers', ['flight_mode_not_auto']),
        ):
            with self.subTest(field=field, value=value):
                status = ready_status()
                status[field] = value
                self.assertIsNotNone(pending_reason(status))


@unittest.skipUnless(os.name == 'posix' and shutil.which('bash'), 'requires bash')
class ShellLifecycleTest(unittest.TestCase):
    def run_fake_stack(self, launch_seconds, cancel):
        # Replace all launch and sender commands with sleeping dummy processes.
        with tempfile.TemporaryDirectory(prefix='manual_target_lifecycle_') as directory:
            root = Path(directory)
            (root / 'scripts').mkdir()
            (root / 'install').mkdir()
            (root / 'install/setup.bash').write_text('')
            (root / 'ros_setup.bash').write_text('')
            bin_dir = root / 'bin'
            bin_dir.mkdir()
            (bin_dir / 'systemctl').write_text('#!/bin/sh\nexit 1\n')
            (root / 'fake_launch.py').write_text(
                'import os,time\nfrom pathlib import Path\n'
                "Path('launch.pid').write_text(str(os.getpid()))\n"
                f'time.sleep({launch_seconds})\n'
            )
            (bin_dir / 'ros2').write_text(
                '#!/bin/sh\nexec python3 "' + str(root / 'fake_launch.py') + '"\n'
            )
            for path in bin_dir.iterdir():
                path.chmod(0o755)
            (root / 'scripts/manual_target_sender.py').write_text(
                'import os,time\nfrom pathlib import Path\n'
                "Path('sender.pid').write_text(str(os.getpid()))\n"
                'time.sleep(60)\n'
            )
            script = Path(__file__).with_name('start_manual_target.sh').read_text()
            script = script.replace('/opt/ros/humble/setup.bash', str(root / 'ros_setup.bash'))
            script = script.replace('/home/nx163/uav_ros2_project', str(root))
            path = root / 'start.sh'
            path.write_text(script)
            env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ['PATH'])
            process = subprocess.Popen(
                ['bash', str(path), '22.8829116', '113.4880722', '0'],
                env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            )
            try:
                deadline = time.monotonic() + 5
                while not (root / 'sender.pid').exists() and time.monotonic() < deadline:
                    time.sleep(0.05)
                self.assertTrue((root / 'sender.pid').exists())
                sender_pid = int((root / 'sender.pid').read_text())
                if cancel:
                    process.send_signal(signal.SIGTERM)
                output, _ = process.communicate(timeout=12)
                self.assertEqual(process.returncode, 143 if cancel else 1, output)
                with self.assertRaises(ProcessLookupError):
                    os.kill(sender_pid, 0)
                self.assertNotIn('Target command published once', output)
            finally:
                if process.poll() is None:
                    process.terminate()
                    process.communicate(timeout=12)

    def test_cancel_stops_pending_sender(self):
        self.run_fake_stack(60, cancel=True)

    def test_launch_exit_cancels_pending_sender(self):
        self.run_fake_stack(0.5, cancel=False)


@unittest.skipUnless(os.environ.get('MANUAL_TARGET_ROS_TEST') == '1', 'opt-in ROS test')
class IsolatedRosTest(unittest.TestCase):
    def test_pending_matching_and_exactly_one_send(self):
        # Refuse to run this dummy readiness publisher in the flight ROS domain.
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'), '1')
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '173')
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from uav_interfaces.msg import TargetCommand

        rclpy.init()
        nodes = []
        try:
            driver = Node('manual_target_test_driver')
            nodes.append(driver)
            recorder_messages = []
            driver.create_subscription(
                TargetCommand, '/vision/target_command', recorder_messages.append, 10,
            )
            status_pub = driver.create_publisher(String, '/mission/safety_status', 10)
            sender = create_sender(22.8829116, 113.4880722, 0.0)
            nodes.append(sender)

            def spin_for(seconds, status=None):
                deadline = time.monotonic() + seconds
                next_publish = 0.0
                while time.monotonic() < deadline:
                    if status is not None and time.monotonic() >= next_publish:
                        status_pub.publish(String(data=json.dumps(status)))
                        next_publish = time.monotonic() + 0.1
                    for node in nodes:
                        rclpy.spin_once(node, timeout_sec=0.01)

            spin_for(2.0, ready_status())
            self.assertGreater(sender.publisher.get_subscription_count(), 0)
            self.assertIsNone(sender.sent_at, 'a recorder must not authorize sending')
            self.assertEqual(recorder_messages, [])

            # An earlier ready snapshot must not trigger a later send when a
            # subscriber appears: another newly received ready status is needed.
            manager = Node('mission_manager_node')
            nodes.append(manager)
            received = []
            manager.create_subscription(
                TargetCommand, '/vision/target_command', received.append, 10,
            )
            spin_for(2.0)
            self.assertIsNone(sender.sent_at)
            blocked = ready_status()
            blocked.update(ready_for_attack=False,
                           attack_start_blockers=['aircraft_not_armed'])
            spin_for(1.0, blocked)
            self.assertEqual(received, [])

            spin_for(2.0, ready_status())
            self.assertIsNotNone(sender.sent_at)
            self.assertEqual(len(received), 1)
            self.assertEqual(len(recorder_messages), 1)
            self.assertAlmostEqual(received[0].latitude, 22.8829116)
            self.assertAlmostEqual(received[0].longitude, 113.4880722)
            self.assertEqual(received[0].heading_deg, 0.0)
            spin_for(1.0, ready_status())
            self.assertEqual(len(received), 1, 'repeated ready status must not resend')
        finally:
            for node in reversed(nodes):
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
