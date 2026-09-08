#!/usr/bin/env python3
"""Hold one manual target in memory until the mission manager is ready."""

import argparse
import json
import math
import time


def pending_reason(status):
    """Require explicit readiness; missing or malformed fields never authorize."""
    if not isinstance(status, dict):
        return 'invalid mission status'
    if status.get('state') != 'STANDBY':
        return 'state=' + str(status.get('state', 'unknown'))
    # ready_for_attack already includes the manager's internal busy check.
    for field in ('attack_executed', 'target_valid'):
        if status.get(field) is not False:
            return field + ' is not false'
    if status.get('ready_for_attack') is not True:
        return 'not ready: ' + str(status.get('attack_start_blockers', 'unknown'))
    if status.get('attack_start_blockers') != []:
        return 'start blockers are not empty'
    return None


def create_sender(latitude, longitude, heading_deg):
    # Deferred imports let the readiness rules be tested without ROS installed.
    from rclpy.node import Node
    from std_msgs.msg import String
    from uav_interfaces.msg import TargetCommand

    class ManualTargetSender(Node):
        def __init__(self):
            super().__init__('manual_target_publisher_node')
            self.target = TargetCommand(
                latitude=latitude, longitude=longitude, heading_deg=heading_deg,
            )
            self.publisher = self.create_publisher(
                TargetCommand, '/vision/target_command', 10,
            )
            # Volatile, depth 1: wait for a new status, not a latched ready flag.
            self.subscription = self.create_subscription(
                String, '/mission/safety_status', self.on_status, 1,
            )
            self.sent_at = None
            self.last_status_at = None
            self.last_reason = None
            self.timer = self.create_timer(5.0, self.check_status)
            self.report('waiting for /mission/safety_status')

        def report(self, reason):
            if reason != self.last_reason:
                self.get_logger().info('[TARGET][PENDING] ' + reason)
                self.last_reason = reason

        def manager_matched(self):
            endpoints = self.get_subscriptions_info_by_topic('/vision/target_command')
            managers = [
                endpoint for endpoint in endpoints
                if endpoint.node_name == 'mission_manager_node'
                and endpoint.node_namespace == '/'
            ]
            # A recorder alone must not satisfy readiness. Conservatively wait
            # for every discovered subscriber to match this persistent publisher.
            return (
                len(managers) == 1
                and self.publisher.get_subscription_count() >= len(endpoints)
            )

        def on_status(self, message):
            if self.sent_at is not None:
                return
            try:
                status = json.loads(message.data)
            except (ValueError, TypeError):
                self.report('invalid mission status JSON')
                return
            self.last_status_at = time.monotonic()
            reason = pending_reason(status)
            if reason is not None:
                self.report(reason)
                return
            if not self.manager_matched():
                self.report('waiting for mission_manager_node target subscription match')
                return
            # Only a newly received ready status can trigger this single send.
            # The manager still rechecks its safety gate when it receives it.
            self.publisher.publish(self.target)
            self.sent_at = time.monotonic()
            self.get_logger().info(
                '[TARGET][SENT] published once after manager readiness: '
                f'latitude={latitude:.7f} longitude={longitude:.7f} '
                f'heading_deg={heading_deg:.2f}; acceptance is checked by manager'
            )

        def check_status(self):
            if self.sent_at is None and (
                self.last_status_at is None
                or time.monotonic() - self.last_status_at > 5.0
            ):
                self.report('no fresh mission status; target remains pending')

    return ManualTargetSender()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('latitude', type=float)
    parser.add_argument('longitude', type=float)
    parser.add_argument('heading_deg', type=float)
    args = parser.parse_args()
    if not all(math.isfinite(value) for value in vars(args).values()):
        parser.error('coordinates and heading must be finite')
    if not -90 <= args.latitude <= 90 or not -180 <= args.longitude <= 180:
        parser.error('coordinates out of range')

    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = None
    try:
        node = create_sender(args.latitude, args.longitude, args.heading_deg)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.sent_at is not None and time.monotonic() - node.sent_at >= 2.0:
                return 0
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
