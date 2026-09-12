import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from uav_interfaces.msg import ReconTarget, TargetCommand


class VisionTargetBridge(Node):
    """Forward one confirmed target to the current auto-execute mission manager."""

    TERMINAL_PHASES = {'manager_accepted_target', 'failed'}

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'vision_target_bridge_node',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('heading_deg', 90.0)
        self.declare_parameter('source_topic', '/vision/recon_result')
        self.declare_parameter('auto_execute', False)
        self.declare_parameter('automation_step_timeout_sec', 8.0)
        self.declare_parameter('automation_total_timeout_sec', 30.0)
        self.declare_parameter('target_retry_interval_sec', 1.0)

        self.heading_deg = float(self.get_parameter('heading_deg').value)
        self.source_topic = str(self.get_parameter('source_topic').value)
        self.auto_execute = bool(self.get_parameter('auto_execute').value)
        self.step_timeout_sec = max(
            1.0,
            float(self.get_parameter('automation_step_timeout_sec').value),
        )
        self.total_timeout_sec = max(
            self.step_timeout_sec,
            float(self.get_parameter('automation_total_timeout_sec').value),
        )
        self.retry_interval_sec = max(
            0.2,
            float(self.get_parameter('target_retry_interval_sec').value),
        )

        if not math.isfinite(self.heading_deg):
            raise ValueError('heading_deg must be finite')

        self.pending_target = None
        self.target_published = False
        self.target_published_monotonic = 0.0
        self.target_publish_attempts = 0
        self.mission_state = 'UNKNOWN'

        self.automation_phase = 'waiting_for_target'
        self.automation_started_monotonic = 0.0
        self.phase_started_monotonic = time.monotonic()

        self.publisher = self.create_publisher(
            TargetCommand,
            '/vision/target_command',
            10,
        )
        self.subscription = self.create_subscription(
            ReconTarget,
            self.source_topic,
            self.target_callback,
            10,
        )

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_subscription = self.create_subscription(
            String,
            '/mission/safety_state',
            self.mission_state_callback,
            state_qos,
        )
        self.automation_timer = self.create_timer(0.2, self.drive_automation)

        self.get_logger().info(
            'Single-target bridge started '
            f'source={self.source_topic} '
            f'heading_deg={self.heading_deg:.6f} '
            f'auto_execute={str(self.auto_execute).lower()} '
            f'step_timeout_sec={self.step_timeout_sec:.1f} '
            f'total_timeout_sec={self.total_timeout_sec:.1f} '
            f'retry_interval_sec={self.retry_interval_sec:.1f}'
        )

    @staticmethod
    def valid_coordinate(latitude, longitude):
        return (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )

    def set_phase(self, phase):
        if phase == self.automation_phase:
            return
        previous = self.automation_phase
        self.automation_phase = phase
        self.phase_started_monotonic = time.monotonic()
        self.get_logger().info(
            f'Automatic fusion phase old={previous} new={phase}'
        )

    def target_callback(self, result):
        if self.target_published or self.pending_target is not None:
            return
        if not result.valid or result.status != 'confirmed':
            return
        if not self.valid_coordinate(result.latitude, result.longitude):
            self.get_logger().warning('Confirmed target has invalid coordinates')
            return

        self.pending_target = result
        self.automation_started_monotonic = time.monotonic()
        self.set_phase(
            'waiting_for_standby'
            if self.auto_execute
            else 'manual_forward_pending'
        )
        self.get_logger().info(
            'Accepted one confirmed recon target '
            f'target_id={result.target_id} '
            f'label={result.label} '
            f'confidence={result.confidence:.6f}'
        )
        self.publish_if_ready()

    def mission_state_callback(self, message):
        self.mission_state = str(message.data)
        if (
            self.auto_execute
            and self.target_published
            and self.automation_phase == 'waiting_for_manager_ack'
        ):
            if self.mission_state == 'EXECUTING':
                self.set_phase('manager_accepted_target')
                self.get_logger().info(
                    'Mission manager accepted target and entered EXECUTING'
                )
            elif self.mission_state == 'SAFE':
                self.automation_failed('mission_manager_entered_SAFE')
        self.publish_if_ready()

    def phase_timed_out(self, now):
        return now - self.phase_started_monotonic > self.step_timeout_sec

    def total_timed_out(self, now):
        return (
            self.automation_started_monotonic > 0.0
            and now - self.automation_started_monotonic > self.total_timeout_sec
        )

    def drive_automation(self):
        if not self.auto_execute or self.pending_target is None:
            return
        if self.automation_phase in self.TERMINAL_PHASES:
            return

        now = time.monotonic()
        if self.total_timed_out(now):
            self.automation_failed(
                f'total_timeout phase={self.automation_phase}'
            )
            return
        if self.mission_state == 'SAFE':
            self.automation_failed('mission_manager_entered_SAFE')
            return

        if self.automation_phase == 'waiting_for_standby':
            self.publish_if_ready()
            if (
                self.automation_phase == 'waiting_for_standby'
                and self.phase_timed_out(now)
            ):
                self.automation_failed(
                    f'STANDBY_timeout state={self.mission_state}'
                )
            return

        if self.automation_phase == 'waiting_for_manager_ack':
            if self.mission_state == 'EXECUTING':
                self.set_phase('manager_accepted_target')
                self.get_logger().info(
                    'Mission manager accepted target and entered EXECUTING'
                )
            elif (
                self.mission_state == 'STANDBY'
                and now - self.target_published_monotonic
                >= self.retry_interval_sec
            ):
                self._publish_pending_target(retry=True)

    def automation_failed(self, reason):
        if self.automation_phase == 'failed':
            return
        previous = self.automation_phase
        self.set_phase('failed')
        self.get_logger().error(
            'Automatic target forwarding stopped '
            f'phase={previous} reason={reason}'
        )

    def publish_if_ready(self):
        if self.pending_target is None:
            return
        if self.automation_phase in self.TERMINAL_PHASES:
            return
        if self.auto_execute:
            if self.automation_phase != 'waiting_for_standby':
                return
            if self.mission_state != 'STANDBY':
                return
        elif self.mission_state != 'STANDBY':
            return

        self._publish_pending_target(retry=False)

    def _publish_pending_target(self, retry):
        result = self.pending_target
        if result is None:
            return
        command = TargetCommand()
        command.latitude = result.latitude
        command.longitude = result.longitude
        command.heading_deg = self.heading_deg
        self.publisher.publish(command)
        self.target_published = True
        self.target_published_monotonic = time.monotonic()
        self.target_publish_attempts += 1
        if self.auto_execute:
            self.set_phase('waiting_for_manager_ack')
        else:
            self.set_phase('target_forwarded_manual')

        self.get_logger().info(
            'Published confirmed target '
            f'target_id={result.target_id} '
            f'label={result.label} '
            f'confidence={result.confidence:.6f} '
            f'lat={command.latitude:.8f} '
            f'lon={command.longitude:.8f} '
            f'heading_deg={command.heading_deg:.2f} '
            f'attempt={self.target_publish_attempts} '
            f'retry={str(bool(retry)).lower()}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VisionTargetBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
