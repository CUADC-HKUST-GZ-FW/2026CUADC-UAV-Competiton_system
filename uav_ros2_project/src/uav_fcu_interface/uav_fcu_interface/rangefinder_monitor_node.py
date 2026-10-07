"""Monitor a MAVROS rangefinder topic and report its measured update rate."""

from collections import deque
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from mavros_msgs.msg import WaypointList, WaypointReached
from mavros_msgs.srv import CommandLong
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32

from uav_fcu_interface.rear_servo_trigger import RearServoTrigger
from uav_fcu_interface.rear_servo_trigger import RearServoTriggerConfig
from uav_fcu_interface.rear_servo_trigger import PenultimateWaypointGate


class FrequencyWindow:
    """Calculate frequency from a bounded sequence of monotonic timestamps."""

    def __init__(self, size):
        self._samples = deque(maxlen=max(2, int(size)))

    def add(self, timestamp):
        """Add a sample timestamp in seconds."""
        self._samples.append(float(timestamp))

    @property
    def count(self):
        """Return the number of samples currently in the window."""
        return len(self._samples)

    @property
    def hz(self):
        """Return the measured frequency, or zero with insufficient data."""
        if len(self._samples) < 2:
            return 0.0
        duration = self._samples[-1] - self._samples[0]
        if duration <= 0.0:
            return 0.0
        return (len(self._samples) - 1) / duration


class RangefinderMonitor(Node):
    """Subscribe to a rangefinder and publish its rate and health."""

    def __init__(self):
        super().__init__('rangefinder_monitor')

        self.declare_parameter('topic', '/mavros/rangefinder_pub')
        self.declare_parameter('rate_topic', '/rangefinder/rate_hz')
        self.declare_parameter('health_topic', '/rangefinder/healthy')
        self.declare_parameter('minimum_rate_hz', 1.0)
        self.declare_parameter('minimum_valid_range_m', 0.0)
        self.declare_parameter('timeout_sec', 1.0)
        self.declare_parameter('report_period_sec', 5.0)
        self.declare_parameter('window_size', 100)
        self.declare_parameter('rear_servo_trigger_distance_m', 0.10)
        self.declare_parameter('rear_servo_confirm_count', 3)
        self.declare_parameter('rear_servo_closed_pwm', 1000)
        self.declare_parameter('rear_servo_open_pwm', 2000)
        self.declare_parameter('rear_servo_channel', 8)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('enable_real_rear_servo', False)
        self.declare_parameter('require_penultimate_waypoint_gate', True)
        self.declare_parameter(
            'mission_waypoints_topic', '/mavros/mission/waypoints'
        )
        self.declare_parameter(
            'mission_reached_topic', '/mavros/mission/reached'
        )
        self.declare_parameter(
            'rear_servo_command_service', '/mavros/cmd/command'
        )

        self.topic = self.get_parameter('topic').value
        self.minimum_rate_hz = float(
            self.get_parameter('minimum_rate_hz').value
        )
        self.minimum_valid_range_m = float(
            self.get_parameter('minimum_valid_range_m').value
        )
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        report_period = float(
            self.get_parameter('report_period_sec').value
        )
        window_size = int(self.get_parameter('window_size').value)

        rear_servo_config = RearServoTriggerConfig(
            trigger_distance_m=float(
                self.get_parameter('rear_servo_trigger_distance_m').value
            ),
            confirm_count=int(
                self.get_parameter('rear_servo_confirm_count').value
            ),
            closed_pwm=int(
                self.get_parameter('rear_servo_closed_pwm').value
            ),
            open_pwm=int(
                self.get_parameter('rear_servo_open_pwm').value
            ),
            channel=int(self.get_parameter('rear_servo_channel').value),
            range_timeout_s=self.timeout_sec,
        )
        rear_servo_config.validate()
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.enable_real_rear_servo = bool(
            self.get_parameter('enable_real_rear_servo').value
        )
        self.require_penultimate_waypoint_gate = bool(
            self.get_parameter('require_penultimate_waypoint_gate').value
        )
        command_service = str(
            self.get_parameter('rear_servo_command_service').value
        )

        self.frequency = FrequencyWindow(window_size)
        self.last_sample_time = None
        self.last_range_m = math.nan
        self.last_sample_valid = False
        self.total_samples = 0

        self.command_client = self.create_client(CommandLong, command_service)
        self.closed_command_attempted = False
        self.closed_state_confirmed = self.dry_run
        self.rear_servo = RearServoTrigger(
            rear_servo_config,
            dry_run=self.dry_run,
            real_control_enabled=self.enable_real_rear_servo,
            command_sender=self._send_rear_servo_pwm,
        )
        self.waypoint_gate = PenultimateWaypointGate()

        self.rate_publisher = self.create_publisher(
            Float32,
            self.get_parameter('rate_topic').value,
            10,
        )
        self.health_publisher = self.create_publisher(
            Bool,
            self.get_parameter('health_topic').value,
            10,
        )
        self.subscription = self.create_subscription(
            Range,
            self.topic,
            self.range_callback,
            qos_profile_sensor_data,
        )
        mission_list_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.waypoint_subscription = self.create_subscription(
            WaypointList,
            str(self.get_parameter('mission_waypoints_topic').value),
            self.mission_waypoints_callback,
            mission_list_qos,
        )
        self.waypoint_reached_subscription = self.create_subscription(
            WaypointReached,
            str(self.get_parameter('mission_reached_topic').value),
            self.mission_reached_callback,
            10,
        )
        self.timer = self.create_timer(report_period, self.report)
        watchdog_period = min(0.25, max(0.05, self.timeout_sec / 2.0))
        self.watchdog_timer = self.create_timer(
            watchdog_period, self.check_range_timeout
        )
        self.closed_state_timer = self.create_timer(
            0.2, self.initialize_rear_servo_closed
        )

        self.get_logger().info(
            f'Monitoring {self.topic}; minimum rate '
            f'{self.minimum_rate_hz:.2f} Hz, minimum range '
            f'{self.minimum_valid_range_m:.3f} m, '
            f'timeout {self.timeout_sec:.2f} s'
        )
        self.get_logger().info(
            '[REAR_SERVO] initialized\n'
            f'  threshold: {rear_servo_config.trigger_distance_m:.3f} m\n'
            f'  confirm_count: {rear_servo_config.confirm_count}\n'
            f'  channel: {rear_servo_config.channel}\n'
            f'  closed_pwm: {rear_servo_config.closed_pwm}\n'
            f'  open_pwm: {rear_servo_config.open_pwm}\n'
            f'  dry_run: {str(self.dry_run).lower()}\n'
            '  waypoint_gate: '
            f'{str(self.require_penultimate_waypoint_gate).lower()}\n'
            '  startup_command_sent: false'
        )

    @staticmethod
    def _mission_signature(waypoints):
        """Return a stable signature that changes when the FCU mission does."""
        return tuple(
            (
                int(waypoint.frame),
                int(waypoint.command),
                bool(waypoint.autocontinue),
                round(float(waypoint.param1), 6),
                round(float(waypoint.param2), 6),
                round(float(waypoint.param3), 6),
                round(float(waypoint.param4), 6),
                round(float(waypoint.x_lat), 7),
                round(float(waypoint.y_long), 7),
                round(float(waypoint.z_alt), 3),
            )
            for waypoint in waypoints
        )

    def mission_waypoints_callback(self, msg):
        """Recompute the penultimate target after an FCU mission change."""
        event = self.waypoint_gate.observe_mission(
            self._mission_signature(msg.waypoints),
            len(msg.waypoints),
            msg.current_seq,
        )
        if event is None:
            return
        self.rear_servo.observe(math.nan, False, time.monotonic())
        target = self.waypoint_gate.target_seq
        self.get_logger().info(
            '[REAR_SERVO] waypoint gate mission updated '
            f'count={len(msg.waypoints)} '
            f'penultimate_seq={target} current_seq={msg.current_seq} '
            f'gate_open={str(self.waypoint_gate.is_open).lower()}'
        )

    def mission_reached_callback(self, msg):
        """Open the gate only after the current mission's penultimate item."""
        if not self.waypoint_gate.observe_reached(msg.wp_seq):
            return
        self.get_logger().info(
            '[REAR_SERVO] waypoint gate OPENED '
            f'reached_seq={msg.wp_seq} '
            f'penultimate_seq={self.waypoint_gate.target_seq} '
            f'mission_count={self.waypoint_gate.mission_count}'
        )

    def range_callback(self, msg):
        """Record one rangefinder sample."""
        now = time.monotonic()
        self.frequency.add(now)
        self.last_sample_time = now
        self.last_range_m = float(msg.range)
        self.last_sample_valid = (
            math.isfinite(self.last_range_m)
            and self.minimum_valid_range_m <= self.last_range_m
            and self.last_range_m <= float(msg.max_range)
        )
        self.total_samples += 1

        rate_is_healthy = (
            self.frequency.count < 2
            or self.frequency.hz >= self.minimum_rate_hz
        )
        range_data_valid = self.last_sample_valid and rate_is_healthy
        control_path_ready = self.dry_run or (
            self.enable_real_rear_servo
            and self.closed_state_confirmed
            and self.command_client.service_is_ready()
        )
        waypoint_gate_ready = (
            not self.require_penultimate_waypoint_gate
            or self.waypoint_gate.is_open
        )
        range_data_valid = (
            range_data_valid
            and control_path_ready
            and waypoint_gate_ready
        )
        event = self.rear_servo.observe(
            self.last_range_m,
            range_data_valid,
            now,
        )
        self._handle_rear_servo_event(event)

    def _handle_rear_servo_event(self, event):
        if event is None:
            return
        if event.kind == 'candidate':
            self.get_logger().info(
                f'[REAR_SERVO] distance={event.distance_m:.3f} m '
                f'confirm={event.confirm_count}/'
                f'{self.rear_servo.config.confirm_count}'
            )
            return
        if event.kind.startswith('reset_'):
            self.get_logger().debug(
                f'[REAR_SERVO] confirmation reset: {event.kind}'
            )
            return

        self.get_logger().info(
            '[REAR_SERVO] TRIGGERED\n'
            f'  distance: {event.distance_m:.3f} m\n'
            f'  threshold: '
            f'{self.rear_servo.config.trigger_distance_m:.3f} m\n'
            f'  confirm: {event.confirm_count}/'
            f'{self.rear_servo.config.confirm_count}\n'
            f'  pwm: {self.rear_servo.config.open_pwm}'
        )
        if event.action == 'dry_run':
            self.get_logger().info(
                '[REAR_SERVO] DRY_RUN would set '
                f'PWM={self.rear_servo.config.open_pwm}'
            )
        elif event.action == 'blocked':
            self.get_logger().error(
                '[REAR_SERVO] real command blocked by safety gate'
            )
        elif event.action == 'command_failed':
            self.get_logger().error(
                '[REAR_SERVO] failed to dispatch MAV_CMD_DO_SET_SERVO'
            )

    def initialize_rear_servo_closed(self):
        """Send the startup close command once, only through the real gate."""
        if self.dry_run or not self.enable_real_rear_servo:
            return
        if self.closed_command_attempted:
            return
        if not self.command_client.service_is_ready():
            return

        self.closed_command_attempted = True
        dispatched = self._send_rear_servo_pwm(
            self.rear_servo.config.channel,
            self.rear_servo.config.closed_pwm,
            purpose='initialize_closed',
        )
        if dispatched:
            self.get_logger().info(
                '[REAR_SERVO] startup close command dispatched once '
                f'PWM={self.rear_servo.config.closed_pwm}'
            )

    def _send_rear_servo_pwm(self, channel, pwm, purpose='open'):
        if not self.command_client.service_is_ready():
            self.get_logger().error(
                '[REAR_SERVO] /mavros/cmd/command service is not ready'
            )
            return False

        request = CommandLong.Request()
        request.broadcast = False
        request.command = 183
        request.confirmation = 0
        request.param1 = float(channel)
        request.param2 = float(pwm)
        request.param3 = 0.0
        request.param4 = 0.0
        request.param5 = 0.0
        request.param6 = 0.0
        request.param7 = 0.0
        future = self.command_client.call_async(request)
        future.add_done_callback(
            lambda completed: self._rear_servo_command_done(
                completed, purpose
            )
        )
        return True

    def _rear_servo_command_done(self, future, purpose):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'[REAR_SERVO] command service failed: {exc}'
            )
            return
        if response.success:
            if purpose == 'initialize_closed':
                self.closed_state_confirmed = True
            self.get_logger().info(
                '[REAR_SERVO] MAV_CMD_DO_SET_SERVO accepted by FCU '
                f'purpose={purpose}'
            )
        else:
            self.get_logger().error(
                '[REAR_SERVO] MAV_CMD_DO_SET_SERVO rejected by FCU '
                f'result={response.result}'
            )

    def check_range_timeout(self):
        if self.rear_servo.check_timeout(time.monotonic()):
            self.get_logger().debug(
                '[REAR_SERVO] confirmation reset: range data timeout'
            )

    def report(self):
        """Publish status and log a concise measurement summary."""
        now = time.monotonic()
        age = math.inf
        if self.last_sample_time is not None:
            age = now - self.last_sample_time

        rate_hz = self.frequency.hz
        healthy = (
            age <= self.timeout_sec
            and rate_hz >= self.minimum_rate_hz
            and self.last_sample_valid
        )

        self.rate_publisher.publish(Float32(data=float(rate_hz)))
        self.health_publisher.publish(Bool(data=healthy))

        if self.last_sample_time is None:
            self.get_logger().warning(
                f'No range data received from {self.topic}'
            )
            return

        message = (
            f'topic={self.topic} rate={rate_hz:.2f}Hz '
            f'range={self.last_range_m:.3f}m age={age:.3f}s '
            f'samples={self.total_samples} healthy={healthy} '
            f'waypoint_gate_open={self.waypoint_gate.is_open} '
            f'penultimate_seq={self.waypoint_gate.target_seq}'
        )
        if healthy:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)


def main(args=None):
    """Run the rangefinder monitor node."""
    rclpy.init(args=args)
    node = RangefinderMonitor()
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
