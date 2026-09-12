"""ROS 2只读载荷监控节点；仅订阅遥测并发布诊断信息."""

import json
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from mavros_msgs.msg import RCOut, State, WaypointList, WaypointReached
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .payload_monitor import PayloadMonitor, PayloadMonitorConfig, PayloadMonitorState


class PayloadMonitorNode(Node):
    """把MAVROS只读消息交给纯逻辑状态机，并输出去重、节流日志."""

    def __init__(self):
        super().__init__('payload_monitor_node')

        self.declare_parameter('enabled', True)
        self.declare_parameter('servo_channel', 7)
        self.declare_parameter('release_pwm', 1900)
        self.declare_parameter('pwm_tolerance_us', 20)
        self.declare_parameter('required_consecutive_samples', 3)
        self.declare_parameter('telemetry_stale_timeout_s', 1.0)
        self.declare_parameter('execution_timeout_s', 5.0)
        self.declare_parameter('status_log_period_s', 2.0)
        self.declare_parameter('warning_throttle_s', 5.0)
        self.declare_parameter('waypoint_list_topic', '/mavros/mission/waypoints')
        self.declare_parameter('waypoint_reached_topic', '/mavros/mission/reached')
        self.declare_parameter('rc_out_topic', '/mavros/rc/out')
        self.declare_parameter('state_topic', '/mavros/state')
        self.declare_parameter('task_id_topic', '/mission/active_task_id')
        self.declare_parameter('mission_event_topic', '/fcu/mission_summary_event')
        self.declare_parameter('status_topic', '/payload/monitor/status')

        self.enabled = bool(self.get_parameter('enabled').value)
        self.status_log_period_s = self._positive_float('status_log_period_s')
        config = PayloadMonitorConfig(
            servo_channel=int(self.get_parameter('servo_channel').value),
            release_pwm=int(self.get_parameter('release_pwm').value),
            pwm_tolerance_us=int(self.get_parameter('pwm_tolerance_us').value),
            required_consecutive_samples=int(
                self.get_parameter('required_consecutive_samples').value
            ),
            telemetry_stale_timeout_s=self._positive_float('telemetry_stale_timeout_s'),
            execution_timeout_s=self._positive_float('execution_timeout_s'),
            warning_throttle_s=self._positive_float('warning_throttle_s'),
        )
        config.validate()
        self.monitor = PayloadMonitor(config)

        self.mavros_connected = False
        self.armed = False
        self.flight_mode = ''
        self._last_status_log_time = None
        self._last_waypoint_list = None

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        task_qos = QoSProfile(depth=1)
        task_qos.reliability = ReliabilityPolicy.RELIABLE
        task_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # 以下全部为订阅器；本节点没有MAVROS服务客户端或控制发布器。
        self.create_subscription(
            WaypointList,
            str(self.get_parameter('waypoint_list_topic').value),
            self._waypoint_list_callback,
            10,
        )
        self.create_subscription(
            WaypointReached,
            str(self.get_parameter('waypoint_reached_topic').value),
            self._waypoint_reached_callback,
            10,
        )
        self.create_subscription(
            RCOut,
            str(self.get_parameter('rc_out_topic').value),
            self._rc_out_callback,
            sensor_qos,
        )
        self.create_subscription(
            State,
            str(self.get_parameter('state_topic').value),
            self._state_callback,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('task_id_topic').value),
            self._task_id_callback,
            task_qos,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('mission_event_topic').value),
            self._mission_event_callback,
            10,
        )

        self.status_publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            '[PAYLOAD][task=unknown][state=IDLE] monitor started '
            f'enabled={str(self.enabled).lower()} channel={config.servo_channel} '
            f'expected_pwm={config.release_pwm} read_only=true'
        )

    def _positive_float(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name} must be a finite value > 0')
        return value

    def _now(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _task_id_callback(self, msg):
        if not self.enabled:
            return
        previous_task = self.monitor.task_id
        events = self.monitor.set_task_id(msg.data, self._now())
        self._log_events(events)
        if self.monitor.task_id != previous_task:
            self.get_logger().info(
                self._prefix() + ' active task updated; waiting for verified mission upload'
            )

    def _mission_event_callback(self, msg):
        if not self.enabled:
            return
        try:
            event = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if event.get('event') != 'composite_mission_uploaded':
            return
        if not bool(event.get('success')) or not bool(event.get('verified')):
            return
        event_task_id = str(event.get('task_id', '')).strip()
        if event_task_id in {'', 'none', 'unknown'}:
            return
        if event_task_id != self.monitor.task_id:
            self.get_logger().warning(
                self._prefix()
                + ' verified mission event ignored reason=task_id_mismatch '
                f'event_task_id={event_task_id}'
            )
            return
        self.monitor.start_mission_monitoring(self._now())
        self.get_logger().info(
            self._prefix(PayloadMonitorState.WAITING_FOR_COMMAND_UPLOAD)
            + ' verified composite upload received; monitoring enabled'
        )
        if self._last_waypoint_list is not None:
            events = self.monitor.observe_mission(
                self._last_waypoint_list.waypoints,
                self._last_waypoint_list.current_seq,
                self._now(),
            )
            self._log_events(events)

    def _waypoint_list_callback(self, msg):
        if not self.enabled:
            return
        self._last_waypoint_list = msg
        events = self.monitor.observe_mission(msg.waypoints, msg.current_seq, self._now())
        self._log_events(events)

    def _waypoint_reached_callback(self, msg):
        if not self.enabled:
            return
        events = self.monitor.observe_waypoint_reached(msg.wp_seq, self._now())
        self._log_events(events)

    def _rc_out_callback(self, msg):
        if not self.enabled:
            return
        events = self.monitor.observe_rc_out(msg.channels, self._now())
        self._log_events(events)

    def _state_callback(self, msg):
        was_connected = self.mavros_connected
        previous_mode = self.flight_mode
        self.mavros_connected = bool(msg.connected)
        self.armed = bool(msg.armed)
        self.flight_mode = str(msg.mode)

        if self.enabled and was_connected and not self.mavros_connected:
            self.get_logger().warning(self._prefix() + ' MAVROS disconnected')
        if (
            self.enabled
            and previous_mode == 'AUTO'
            and self.flight_mode
            and self.flight_mode != 'AUTO'
            and self.monitor.state not in (
                PayloadMonitorState.IDLE,
                PayloadMonitorState.PWM_CONFIRMED,
            )
        ):
            self.get_logger().warning(
                self._prefix() + f' flight mode changed mode={self.flight_mode} expected=AUTO'
            )

    def _tick(self):
        if not self.enabled:
            return
        now = self._now()
        self._log_events(self.monitor.tick(now))
        self._publish_diagnostics()

        if self.monitor.state == PayloadMonitorState.IDLE:
            return
        if (
            self._last_status_log_time is None
            or now - self._last_status_log_time >= self.status_log_period_s
        ):
            pwm = 'none' if self.monitor.observed_pwm is None else self.monitor.observed_pwm
            seq = (
                'none'
                if self.monitor.last_current_seq is None
                else self.monitor.last_current_seq
            )
            release_seq = (
                'none'
                if self.monitor.release_command_seq is None
                else self.monitor.release_command_seq
            )
            self.get_logger().info(
                f'[STATUS][task={self.monitor.task_id}] '
                f'payload_state={self.monitor.state} release_seq={release_seq} '
                f'current_seq={seq} pwm={pwm} '
                'release_confirmed_latched='
                f'{str(self.monitor.release_confirmed_latched).lower()} '
                f'mavros_connected={str(self.mavros_connected).lower()} '
                f'mode={self.flight_mode or "unknown"}'
            )
            self._last_status_log_time = now

    def _prefix(self, state=None):
        return (
            f'[PAYLOAD][task={self.monitor.task_id}]'
            f'[state={state or self.monitor.state}]'
        )

    def _log_events(self, events):
        for event in events:
            message = self._prefix(event.state) + ' ' + event.message
            if event.level == 'error':
                self.get_logger().error(message)
            elif event.level == 'warning':
                self.get_logger().warning(message)
            else:
                self.get_logger().info(message)
            if event.key == 'command_uploaded':
                self.get_logger().info(
                    f'[PAYLOAD][task={self.monitor.task_id}] '
                    'COMMAND_UPLOADED -> WAITING_FOR_COMMAND_REACHED'
                )
            elif event.key == 'command_reached':
                self.get_logger().info(
                    f'[PAYLOAD][task={self.monitor.task_id}] '
                    'COMMAND_REACHED -> WAITING_FOR_PWM_CONFIRMATION'
                )

    @staticmethod
    def _key(name, value):
        return KeyValue(key=name, value=str(value))

    def _publish_diagnostics(self):
        status = DiagnosticStatus()
        status.name = 'uav_payload/payload_monitor'
        status.hardware_id = 'mavros-read-only'
        status.level = (
            DiagnosticStatus.ERROR
            if self.monitor.state in (PayloadMonitorState.TIMEOUT, PayloadMonitorState.INVALID)
            else DiagnosticStatus.OK
        )
        status.message = self.monitor.state
        status.values = [
            self._key('task_id', self.monitor.task_id),
            self._key('mission_epoch', self.monitor.mission_epoch),
            self._key('state', self.monitor.state),
            self._key('release_command_seq', self.monitor.release_command_seq),
            self._key('command_uploaded', self.monitor.release_command_seen),
            self._key('command_reached', self.monitor.release_command_reached),
            self._key('pwm_confirmed', self.monitor.release_pwm_confirmed),
            self._key(
                'release_confirmed_latched',
                self.monitor.release_confirmed_latched,
            ),
            self._key('execution_window_armed', self.monitor.execution_window_armed),
            self._key('servo_channel', self.monitor.config.servo_channel),
            self._key('expected_pwm', self.monitor.config.release_pwm),
            self._key('observed_pwm', self.monitor.observed_pwm),
            self._key('mavros_connected', self.mavros_connected),
            self._key('flight_mode', self.flight_mode),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.status_publisher.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PayloadMonitorNode()
        rclpy.spin(node)
    except ValueError as exc:
        if node is not None:
            node.get_logger().fatal(
                f'[PAYLOAD][task=unknown][state=INVALID] '
                f'invalid configuration reason={exc!r}'
            )
        else:
            rclpy.logging.get_logger('payload_monitor_node').fatal(
                f'[PAYLOAD][task=unknown][state=INVALID] '
                f'invalid configuration reason={exc!r}'
            )
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
