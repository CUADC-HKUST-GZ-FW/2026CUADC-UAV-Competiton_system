"""只读实飞/仿真任务摘要日志节点。

本节点只订阅 ROS 2 话题并写入本地人类可读日志文件，
不向 MAVROS、飞控或载荷发送任何控制命令。
"""

from datetime import datetime, timezone
import json
import math
import os
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import State, VfrHud, WaypointList, WaypointReached
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64, String

from uav_interfaces.msg import TargetCommand


class FlightSummaryLoggerNode(Node):
    """把关键任务事件写入精简的人类可读 flight_summary.log。"""

    MAV_CMD_DO_SET_SERVO = 183

    def __init__(self):
        super().__init__('flight_summary_logger_node')

        self.declare_parameter(
            'summary_path',
            os.environ.get('UAV_FLIGHT_SUMMARY_PATH', ''),
        )
        self.declare_parameter('enabled', True)
        self.declare_parameter('c_distance_log_period_s', 2.0)
        self.declare_parameter('c_distance_improvement_step_m', 1.0)
        self.declare_parameter('aburcd_update_metrics_enabled', True)
        self.declare_parameter('dynamic_r_enabled', True)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.summary_path = self._resolve_summary_path(
            str(self.get_parameter('summary_path').value)
        )
        self.c_distance_log_period_s = max(
            0.2,
            float(self.get_parameter('c_distance_log_period_s').value),
        )
        self.c_distance_improvement_step_m = max(
            0.0,
            float(self.get_parameter('c_distance_improvement_step_m').value),
        )
        self.aburcd_update_metrics_enabled = bool(
            self.get_parameter('aburcd_update_metrics_enabled').value
        )
        self.dynamic_r_enabled = bool(
            self.get_parameter('dynamic_r_enabled').value
        )

        self._lock = threading.Lock()
        self.active_task_id = 'none'
        self.safety_state = 'unknown'
        self.last_state = None
        self.last_gps = None
        self.last_relative_alt_m = None
        self.last_vfr = None
        self.target = None
        # Actual ROS publisher-node tracking for /vision/target_command.
        # ROS 2 Humble rclpy subscription callbacks do not expose publisher GID,
        # so we resolve the authoritative publisher through the ROS graph.
        # The launch/start scripts are designed to keep exactly one publisher
        # for this command topic. A short cache also covers one-shot CLI publishers.
        self.target_topic = '/vision/target_command'
        self._target_publisher_cache = []
        self._target_publisher_cache_monotonic = 0.0
        self._target_publisher_cache_max_age_sec = 5.0
        self.r_point = None
        self.nearest_c_distance_m = None
        self.nearest_c_sample = None
        self._last_c_distance_log_time = 0.0
        self._last_logged_c_distance_m = None
        self._mission_signature = None
        self.a_seq = None
        self.u_seq = None
        self.release_seq = None
        self.r_seq = None
        self.d_seq = None
        self._r_reached_logged = False
        self._payload_flags = {}
        self._last_health_status_log_time = 0.0
        self._last_health_status_signature = None
        self._aburcd_metrics = {}

        os.makedirs(os.path.dirname(self.summary_path), exist_ok=True)
        self.aburcd_metrics_path = os.path.join(
            os.path.dirname(self.summary_path),
            'aburcd_update_metrics.jsonl',
        )

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        task_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            String,
            '/mission/safety_state',
            self.safety_state_callback,
            10,
        )
        self.create_subscription(
            String,
            '/mission/safety_status',
            self.safety_status_callback,
            10,
        )
        self.create_subscription(
            String,
            '/mission/active_task_id',
            self.task_id_callback,
            task_qos,
        )
        self.create_subscription(
            TargetCommand,
            '/vision/target_command',
            self.target_callback,
            10,
        )
        self.create_subscription(
            String,
            '/fcu/mission_summary_event',
            self.fcu_mission_event_callback,
            10,
        )
        self.create_subscription(
            String,
            '/fcu/composite_mission_complete',
            self.composite_complete_callback,
            10,
        )
        self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoints_callback,
            10,
        )
        self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
            10,
        )
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            sensor_qos,
        )
        self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.relative_alt_callback,
            sensor_qos,
        )
        self.create_subscription(
            VfrHud,
            '/mavros/vfr_hud',
            self.vfr_callback,
            sensor_qos,
        )
        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray,
            '/payload/monitor/status',
            self.payload_status_callback,
            10,
        )

        # Refresh often enough to see short-lived `ros2 topic pub --once` nodes.
        self.create_timer(0.2, self._refresh_target_publisher_cache)

        self.write_event(
            'summary_logger_started',
            summary_path=self.summary_path,
            run_dir=os.environ.get('UAV_RUN_DIR', ''),
            read_only=True,
        )
        self.get_logger().info(
            f'[SUMMARY][state=STARTUP] flight summary logger started '
            f'path={self.summary_path} read_only=true'
        )

    def _resolve_summary_path(self, configured_path):
        configured = configured_path.strip()
        if configured:
            configured = os.path.abspath(os.path.expanduser(configured))
            root, _ext = os.path.splitext(configured)
            return root + '.log'

        run_dir = os.environ.get('UAV_RUN_DIR')
        if run_dir:
            return os.path.join(
                run_dir,
                '02_mission_summary',
                'flight_summary.log',
            )

        return os.path.abspath(
            os.path.expanduser('~/uav_flight_logs/flight_summary.log')
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def wall_time():
        return datetime.now(timezone.utc).astimezone().isoformat(
            timespec='milliseconds'
        )

    def write_event(self, event, **fields):
        if not self.enabled:
            return

        record = {
            'time': self.wall_time(),
            'event': event,
            'task_id': self.active_task_id,
        }
        record.update(fields)

        text = self._format_human_event(record)

        with self._lock:
            with open(
                self.summary_path,
                'a',
                encoding='utf-8',
            ) as stream:
                stream.write(text)
                stream.write('\n')
            if (
                self.aburcd_update_metrics_enabled
                and event in self._ABURCD_EVENTS
            ):
                metric_record = {
                    key: record.get(key)
                    for key in self._ABURCD_METRIC_FIELDS
                }
                with open(
                    self.aburcd_metrics_path,
                    'a',
                    encoding='utf-8',
                ) as stream:
                    stream.write(json.dumps(metric_record, ensure_ascii=False))
                    stream.write('\n')

    _ABURCD_EVENTS = {
        'second_full_verify_start', 'second_full_verify_pass',
        'second_full_pull_failed', 'dynamic_update_skipped',
        'dynamic_update_too_late', 'dynamic_mission_structure_mismatch',
        'mission_progress_recovery', 'mission_progress_unknown',
        'mission_progress_check', 'mission_progress_safe', 'mission_progress_accepted',
        'mission_state_unknown', 'dynamic_mission_failed',
        'mission_current_a', 'a_reached', 'b_crossed', 'b_state_frozen',
        'r_calc_start', 'r_calc_done', 'r_calc_failed', 'second_full_push_start',
        'second_full_push_done', 'second_full_pull_start', 'second_full_pull_done', 'second_full_push_failed',
        'dynamic_mission_verified', 'dynamic_update_cancelled',
        'mission_inconsistent', 'r_dynamic_out_of_range',
        'mission_current_u', 'u_reached',
        'mission_current_r', 'r_reached', 'r_update_rejected_late',
        'r_update_rejected_mission_busy',
        'error_r_active_before_update_verified',
    }
    _ABURCD_METRIC_FIELDS = (
        'time', 'event', 'task_id', 'current_seq', 'reached_seq',
        'ground_speed_mps', 'vertical_speed_mps', 'heading_deg',
        'lat', 'lon', 'altitude_m', 'a_seq', 'u_seq', 'r_seq',
        'release_seq', 'd_seq', 'distance_to_u_m', 'distance_to_r_m',
        'snapshot_latency_ms', 'calc_duration_ms', 'push_duration_ms',
        'verify_duration_ms', 'dynamic_update_total_ms',
        'r_commit_margin_sec', 'r_commit_margin_m', 'failure_reason',
        'dynamic_trigger_timestamp', 'r_calc_start', 'r_calc_done',
        'second_full_push_start', 'second_full_push_done',
        'second_full_push_duration_ms', 'second_full_pull_start',
        'second_full_pull_done', 'second_full_pull_duration_ms',
        'second_full_verify_start', 'second_full_verify_done',
        'verify_cpu_duration_ms', 'dynamic_full_update_total_ms',
        'current_seq_before_update', 'current_seq_after_update',
        'last_reached_seq_before_update', 'last_reached_seq_after_update',
        'mission_state', 'r_source', 'reason', 'push_pass', 'action',
        'dynamic_update_state',
        'deadline_current_seq', 'deadline_last_reached_seq',
        'dynamic_worker_cancel_reason',
    )
    _ABURCD_HUMAN_FIELDS = (
        'current_seq', 'reached_seq', 'r_seq', 'snapshot_latency_ms',
        'calc_duration_ms', 'push_duration_ms', 'verify_duration_ms',
        'dynamic_update_total_ms', 'r_commit_margin_sec',
        'r_commit_margin_m', 'failure_reason', 'mission_state', 'r_source',
        'reason', 'dynamic_worker_cancel_reason', 'action', 'dynamic_update_state',
        'current_seq_before_update', 'last_reached_seq_before_update',
        'current_seq_after_update', 'last_reached_seq_after_update',
    )

    def _format_human_event(self, record):
        timestamp = str(record.get('time', ''))
        if 'T' in timestamp:
            timestamp = timestamp.split('T', 1)[1]
        if '+' in timestamp:
            timestamp = timestamp.split('+', 1)[0]

        event = str(record.get('event', 'unknown'))
        task_id = str(record.get('task_id', 'none'))

        if event == 'summary_logger_started':
            run_dir = str(record.get('run_dir', ''))
            run_name = os.path.basename(run_dir.rstrip('/')) if run_dir else 'unknown'
            return (
                f'[{timestamp}] SYSTEM   Summary logger started\n'
                f'  run: {run_name}\n'
                f'  log: {record.get("summary_path", self.summary_path)}'
            )

        if event == 'safety_state_changed':
            previous = record.get('previous_state', 'unknown')
            state = record.get('state', 'unknown')
            return f'[{timestamp}] SAFETY   {previous} -> {state}'

        if event == 'health_status':
            lines = [f'[{timestamp}] FAILED   Safety / health failure']
            reason = record.get('safe_reason')
            if reason:
                lines.append(f'  reason: {reason}')
            for key, label in (
                ('fcu_connected', 'FCU connected'),
                ('heartbeat_age_sec', 'heartbeat age'),
                ('gps_healthy', 'GPS healthy'),
                ('gps_fix_type', 'GPS fix type'),
                ('gps_satellites', 'GPS satellites'),
                ('ekf_healthy', 'EKF healthy'),
                ('position_age_sec', 'position age'),
            ):
                value = record.get(key)
                if value is None:
                    continue
                if isinstance(value, bool):
                    value = 'YES' if value else 'NO'
                elif isinstance(value, float):
                    value = f'{value:.2f}'
                lines.append(f'  {label}: {value}')
            return '\n'.join(lines)

        if event == 'active_task_changed':
            new_task = str(record.get('task_id', 'none'))
            previous = str(record.get('previous_task_id', 'none'))
            if new_task in {'', 'none'}:
                return f'[{timestamp}] TASK     {previous} completed / cleared'
            return f'[{timestamp}] TASK     {previous} -> {new_task}'

        if event == 'vision_target_received':
            lines = [f'[{timestamp}] TARGET   Target received']
            if task_id not in {'', 'none'}:
                lines.append(f'  task: {task_id}')
            lines.extend([
                f'  lat: {float(record.get("latitude", 0.0)):.7f}',
                f'  lon: {float(record.get("longitude", 0.0)):.7f}',
                f'  heading: {float(record.get("heading_deg", 0.0)):.1f} deg',
                f'  source topic: {record.get("source_topic", self.target_topic)}',
                f'  source node: {record.get("source_node", "UNKNOWN")}',
            ])
            resolution = record.get('source_resolution')
            if resolution:
                lines.append(f'  source resolution: {resolution}')
            publisher_count = record.get('publisher_count')
            if publisher_count is not None:
                lines.append(f'  publisher count: {publisher_count}')
            return '\n'.join(lines)

        if event == 'composite_mission_uploaded':
            return (
                f'[{timestamp}] MISSION  Composite mission uploaded\n'
                f'  original: {record.get("original_count")}  '
                f'composite: {record.get("total_count")}\n'
                f'  A={record.get("a_seq")} '
                f'U={record.get("u_seq")} '
                f'R={record.get("r_seq")} '
                f'RELEASE={record.get("release_seq")} '
                f'D={record.get("d_seq")}\n'
                f'  verified: {"YES" if record.get("verified") else "NO"}'
            )

        if event in {
            'mission_current_a', 'mission_current_u', 'mission_current_r',
            'a_reached', 'u_reached',
        }:
            return f'[{timestamp}] ABURCD  {event.upper()}'

        if event == 'r_reached':
            lines = [f'[{timestamp}] ABURCD  R_REACHED']
            if isinstance(self.r_point, dict):
                lat = self.r_point.get('lat')
                lon = self.r_point.get('lon')
                if lat is not None and lon is not None:
                    lines.extend([
                        f'  planned R lat: {float(lat):.7f}',
                        f'  planned R lon: {float(lon):.7f}',
                    ])
            return '\n'.join(lines)

        if event == 'dynamic_mission_verified':
            return (
                'DYNAMIC R UPDATE\n'
                f'  trigger: {record.get("dynamic_trigger_timestamp")}\n'
                f'  R dynamic: lat={record.get("dynamic_r_lat")} '
                f'lon={record.get("dynamic_r_lon")} alt={record.get("dynamic_r_alt")}\n'
                'SECOND FULL MISSION UPDATE\n'
                f'  push: {"PASS" if record.get("push_pass") else "FAILED (reconciled)"} '
                f'{record.get("second_full_push_duration_ms")} ms\n'
                f'  pull: PASS {record.get("second_full_pull_duration_ms")} ms\n'
                f'  verify: PASS {record.get("verify_cpu_duration_ms")} ms\n'
                'MISSION PROGRESS\n'
                f'  before: seq{record.get("current_seq_before_update")}\n'
                f'  after: seq{record.get("current_seq_after_update")}\n'
                'DYNAMIC R\n  result: VERIFIED\n'
                f'  total: {record.get("dynamic_full_update_total_ms")} ms\n'
                '  source: DYNAMIC'
            )
        if event in {'dynamic_update_skipped', 'dynamic_update_too_late'}:
            return ('DYNAMIC R\n  result: SKIPPED\n'
                    f'  reason: {record.get("reason", record.get("failure_reason"))}\n'
                    '  source: R_SAFE')
        if event == 'dynamic_mission_failed':
            return ('DYNAMIC R\n  result: FAILED\n'
                    f'  mission_state: {record.get("mission_state")}\n'
                    f'  reason: {record.get("reason")}')

        if event in self._ABURCD_EVENTS:
            lines = [f'[{timestamp}] ABURCD  {event.upper()}']
            for key in self._ABURCD_HUMAN_FIELDS:
                value = record.get(key)
                if value is not None:
                    lines.append(f'  {key}: {value}')
            return '\n'.join(lines)

        if event in {'composite_mission_failed', 'composite_mission_rejected'}:
            title = 'FAILED' if event.endswith('failed') else 'REJECTED'
            lines = [f'[{timestamp}] FAILED   Composite mission {title}']
            reason = record.get('reason')
            if reason:
                lines.append(f'  reason: {reason}')
            if 'upload_attempted' in record:
                lines.append(
                    f'  upload attempted: '
                    f'{"YES" if record.get("upload_attempted") else "NO"}'
                )
            return '\n'.join(lines)

        if event == 'composite_mission_dry_run':
            return (
                f'[{timestamp}] MISSION  Composite mission dry-run\n'
                f'  upload attempted: NO\n'
                f'  reason: {record.get("reason", "dry_run_enabled")}'
            )

        if event == 'composite_a_reached':
            return f'[{timestamp}] ATTACK   A reached (seq {record.get("seq")})'

        if event == 'composite_ab_trajectory_check':
            passed = bool(record.get('passed', record.get('success', False)))
            lines = [
                f'[{timestamp}] CHECK    A-B trajectory '
                f'{"PASS" if passed else "FAIL"}'
            ]
            if record.get('reason'):
                lines.append(f'  reason: {record.get("reason")}')
            for key, label, suffix in (
                ('cross_track_m', 'cross track', ' m'),
                ('max_cross_track_m', 'max cross track', ' m'),
                ('heading_error_deg', 'heading error', ' deg'),
                ('cross_track_limit_m', 'cross-track limit', ' m'),
                ('heading_error_limit_deg', 'heading limit', ' deg'),
            ):
                value = record.get(key)
                if isinstance(value, (int, float)) and math.isfinite(float(value)):
                    lines.append(f'  {label}: {float(value):.2f}{suffix}')
            return '\n'.join(lines)

        if event == 'composite_c_passage_confirmed':
            value = record.get('min_distance_m')
            distance_text = (
                f'{float(value):.2f} m'
                if isinstance(value, (int, float)) and math.isfinite(float(value))
                else 'unknown'
            )
            return (
                f'[{timestamp}] TARGET   C passage confirmed\n'
                f'  evidence: {record.get("evidence", "unknown")}\n'
                f'  minimum distance: {distance_text}'
            )

        if event == 'servo_command_reached':
            lines = [f'[{timestamp}] PAYLOAD  SERVO COMMAND REACHED']
            seq = record.get('release_command_seq', record.get('seq'))
            if seq is not None:
                lines.append(f'  seq: {seq}')
            return '\n'.join(lines)

        if event == 'payload_release_confirmed':
            lines = [f'[{timestamp}] PAYLOAD  RELEASE CONFIRMED']
            gps = record.get('gps')
            latitude = None
            longitude = None
            if isinstance(gps, dict):
                latitude = gps.get('latitude')
                longitude = gps.get('longitude')
            lines.extend([
                '  actual release lat: '
                + (
                    f'{float(latitude):.7f}'
                    if isinstance(latitude, (int, float)) else 'unknown'
                ),
                '  actual release lon: '
                + (
                    f'{float(longitude):.7f}'
                    if isinstance(longitude, (int, float)) else 'unknown'
                ),
            ])
            for key, label, suffix in (
                ('relative_altitude_m', 'relative altitude', ' m'),
                ('groundspeed_mps', 'groundspeed', ' m/s'),
                ('airspeed_mps', 'airspeed', ' m/s'),
                ('distance_to_r_m', 'distance to R', ' m'),
            ):
                lines.append(
                    f'  {label}: {self._format_number(record.get(key), suffix)}'
                )
            pwm = record.get('observed_pwm', record.get('expected_pwm'))
            lines.append(f'  PWM: {pwm if pwm is not None else "unknown"}')
            return '\n'.join(lines)

        if event == 'd_reached':
            value = record.get('nearest_c_distance_m')
            distance_text = (
                f'{float(value):.2f} m'
                if isinstance(value, (int, float)) and math.isfinite(float(value))
                else 'unknown'
            )
            return (
                f'[{timestamp}] ATTACK   D reached (seq {record.get("seq")})\n'
                f'  nearest C distance: {distance_text}'
            )

        if event == 'composite_mission_completed':
            base = (
                f'[{timestamp}] RESULT   Attack segment COMPLETE\n'
                f'  A-B trajectory: '
                f'{"PASS" if record.get("ab_track_passed") else "FAIL"}\n'
                f'  C confirmed: '
                f'{"YES" if record.get("c_confirmed") else "NO"}\n'
                f'  C min distance: '
                f'{self._format_number(record.get("c_min_distance_m"), " m")}'
            )
            if not getattr(self, 'dynamic_r_enabled', False):
                return base
            metric = self._aburcd_metrics
            return (
                base
                + '\n\nABURCD UPDATE SUMMARY\n'
                + 'Result: '
                + str(metric.get('result', 'NOT OBSERVED'))
                + '\nTotal B_CROSSED -> DYNAMIC_MISSION_VERIFIED: '
                + self._metric_text(
                    metric.get('dynamic_update_total_ms'), ' ms'
                )
                + '\nR commit time margin: '
                + self._metric_text(metric.get('r_commit_margin_sec'), ' s')
                + '\nR commit distance margin: '
                + self._metric_text(metric.get('r_commit_margin_m'), ' m')
            )

        if event == 'fcu_connection_changed':
            return (
                f'[{timestamp}] FCU      '
                f'{"CONNECTED" if record.get("connected") else "DISCONNECTED"}'
            )

        if event == 'flight_mode_changed':
            return (
                f'[{timestamp}] FLIGHT   Mode '
                f'{record.get("previous_mode", "UNKNOWN")} '
                f'-> {record.get("mode", "UNKNOWN")}'
            )

        if event == 'armed_state_changed':
            return (
                f'[{timestamp}] FLIGHT   '
                f'{"ARMED" if record.get("armed") else "DISARMED"}'
            )

        if event == 'waypoint_reached':
            return f'[{timestamp}] WP       Waypoint seq {record.get("seq")} reached'

        # Unknown FCU summary events are still kept, so future failures/reasons
        # are not silently discarded.
        lines = [f'[{timestamp}] EVENT    {event.replace("_", " ")}']
        for key, value in record.items():
            if key in {'time', 'event', 'task_id', 'raw', 'points'} or value is None:
                continue
            if isinstance(value, bool):
                value = 'YES' if value else 'NO'
            elif isinstance(value, float):
                value = f'{value:.2f}' if math.isfinite(value) else str(value)
            lines.append(f'  {key.replace("_", " ")}: {value}')
        return '\n'.join(lines)

    @staticmethod
    def _format_number(value, suffix=''):
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            return f'{float(value):.2f}{suffix}'
        return 'unknown'

    @staticmethod
    def _metric_text(value, suffix=''):
        if isinstance(value, (int, float)) and math.isfinite(float(value)):
            return f'{float(value):.3f}{suffix}'
        return 'NOT OBSERVED'

    def safety_state_callback(self, msg):
        state = str(msg.data)
        previous = self.safety_state
        self.safety_state = state
        if state == previous:
            return
        self.write_event(
            'safety_state_changed',
            previous_state=previous,
            state=state,
        )

    def safety_status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        state = str(status.get('state', 'unknown'))
        if state == 'SAFE':
            signature = (
                state,
                status.get('fcu_connected'),
                status.get('gps_healthy'),
                status.get('gps_fix_type'),
                status.get('gps_satellites'),
                status.get('ekf_healthy'),
                status.get('ekf_source'),
                status.get('safe_reason'),
            )
            now = self.now_sec()
            if (
                signature == self._last_health_status_signature
                and now - self._last_health_status_log_time < 5.0
            ):
                return
            self._last_health_status_signature = signature
            self._last_health_status_log_time = now
            self.write_event(
                'health_status',
                state=state,
                fcu_connected=status.get('fcu_connected'),
                heartbeat_age_sec=status.get('heartbeat_age_sec'),
                gps_healthy=status.get('gps_healthy'),
                gps_fix_type=status.get('gps_fix_type'),
                gps_satellites=status.get('gps_satellites'),
                gps_h_acc_m=status.get('gps_h_acc_m'),
                ekf_healthy=status.get('ekf_healthy'),
                ekf_source=status.get('ekf_source'),
                position_age_sec=status.get('position_age_sec'),
                safe_reason=status.get('safe_reason'),
                execution_allowed=status.get('execution_allowed'),
                target_valid=status.get('target_valid'),
            )

    def task_id_callback(self, msg):
        task_id = msg.data.strip() or 'none'
        if task_id == self.active_task_id:
            return
        previous = self.active_task_id
        self.active_task_id = task_id
        if task_id == 'none':
            self.a_seq = None
            self.u_seq = None
            self.r_seq = None
            self.release_seq = None
            self.d_seq = None
            self.r_point = None
            self._r_reached_logged = False
            self._payload_flags.clear()
            self._aburcd_metrics.clear()
        self.write_event(
            'active_task_changed',
            previous_task_id=previous,
            task_id=task_id,
        )

    @staticmethod
    def _full_node_name(node_namespace, node_name):
        node_name = str(node_name).strip()
        node_namespace = str(node_namespace).strip()
        if not node_name:
            return ''
        if node_namespace in {'', '/'}:
            return f'/{node_name}'
        return f'{node_namespace.rstrip("/")}/{node_name}'

    def _query_target_publishers(self):
        """Return unique publisher node names currently visible in the ROS graph."""
        try:
            endpoint_infos = self.get_publishers_info_by_topic(self.target_topic)
        except Exception as error:
            self.get_logger().warning(
                f'[SUMMARY][state=MONITORING] target publisher graph query failed '
                f'topic={self.target_topic} error={error!r}'
            )
            return []

        names = []
        for info in endpoint_infos:
            full_name = self._full_node_name(
                getattr(info, 'node_namespace', ''),
                getattr(info, 'node_name', ''),
            )
            if full_name:
                names.append(full_name)
        return sorted(set(names))

    def _refresh_target_publisher_cache(self):
        publishers = self._query_target_publishers()
        if not publishers:
            return
        self._target_publisher_cache = publishers
        self._target_publisher_cache_monotonic = time.monotonic()

    def _target_publisher_snapshot(self):
        """Resolve the actual publisher node for /vision/target_command.

        Humble's Python subscription callback does not expose publisher GID. In
        this project the command topic is intentionally constrained to one
        authoritative publisher, so a unique graph endpoint is authoritative.
        A recent cache covers the short-lived manual ros2cli publisher.
        """
        publishers = self._query_target_publishers()
        resolution = 'live_graph'

        if publishers:
            self._target_publisher_cache = publishers
            self._target_publisher_cache_monotonic = time.monotonic()
        else:
            age = time.monotonic() - self._target_publisher_cache_monotonic
            if (
                self._target_publisher_cache
                and age <= self._target_publisher_cache_max_age_sec
            ):
                publishers = list(self._target_publisher_cache)
                resolution = 'recent_graph_cache'

        if len(publishers) == 1:
            return {
                'source_node': publishers[0],
                'publisher_count': 1,
                'source_resolution': resolution,
            }

        if len(publishers) > 1:
            return {
                'source_node': 'AMBIGUOUS[' + ', '.join(publishers) + ']',
                'publisher_count': len(publishers),
                'source_resolution': resolution + '_multiple_publishers',
            }

        return {
            'source_node': 'UNKNOWN',
            'publisher_count': 0,
            'source_resolution': 'publisher_not_visible',
        }

    def target_callback(self, msg):
        self.target = {
            'latitude': float(msg.latitude),
            'longitude': float(msg.longitude),
            'heading_deg': float(msg.heading_deg),
        }
        self.nearest_c_distance_m = None
        self.nearest_c_sample = None
        self._last_logged_c_distance_m = None
        self._r_reached_logged = False

        publisher = self._target_publisher_snapshot()
        self.write_event(
            'vision_target_received',
            latitude=self.target['latitude'],
            longitude=self.target['longitude'],
            heading_deg=self.target['heading_deg'],
            source_topic=self.target_topic,
            source_node=publisher['source_node'],
            publisher_count=publisher['publisher_count'],
            source_resolution=publisher['source_resolution'],
        )

    def fcu_mission_event_callback(self, msg):
        event = self._parse_event_payload(msg.data)
        name = str(event.pop('event', 'fcu_mission_event'))

        if name == 'composite_mission_uploaded':
            self.a_seq = self._coerce_optional_int(event.get('a_seq'))
            self.u_seq = self._coerce_optional_int(event.get('u_seq'))
            self.r_seq = self._coerce_optional_int(event.get('r_seq'))
            self.release_seq = self._coerce_optional_int(event.get('release_seq'))
            self.d_seq = self._coerce_optional_int(event.get('d_seq'))

            points = event.get('points')
            if isinstance(points, dict):
                r_point = points.get('R')
                if isinstance(r_point, dict):
                    try:
                        self.r_point = {
                            'lat': float(r_point['lat']),
                            'lon': float(r_point['lon']),
                        }
                    except (KeyError, TypeError, ValueError):
                        self.r_point = None

        if name == 'dynamic_mission_verified':
            try:
                self.r_point = {
                    'lat': float(event['dynamic_r_lat']),
                    'lon': float(event['dynamic_r_lon']),
                }
            except (KeyError, TypeError, ValueError):
                pass

        if name in self._ABURCD_EVENTS:
            for key in (
                *self._ABURCD_METRIC_FIELDS,
            ):
                if event.get(key) is not None:
                    self._aburcd_metrics[key] = event[key]
            if name == 'dynamic_mission_verified':
                self._aburcd_metrics['result'] = 'DYNAMIC_R'
            elif name in {
                'r_calc_failed', 'r_dynamic_out_of_range',
                'dynamic_update_skipped',
            }:
                self._aburcd_metrics['result'] = 'R_SAFE_FALLBACK'
            elif name in {
                'second_full_push_failed', 'r_update_rejected_mission_busy',
                'mission_inconsistent', 'mission_state_unknown',
                'mission_progress_unknown', 'dynamic_mission_failed'
            }:
                self._aburcd_metrics['result'] = 'UPDATE_FAILED'
            elif name in {
                'r_update_rejected_late', 'dynamic_update_too_late',
                'error_r_active_before_update_verified',
            }:
                self._aburcd_metrics['result'] = 'UPDATE_TOO_LATE'

        self.write_event(name, **event)

    def composite_complete_callback(self, msg):
        # Completion is summarized through /fcu/mission_summary_event.
        return

    def waypoints_callback(self, msg):
        commands = [int(wp.command) for wp in msg.waypoints]
        signature = (
            len(msg.waypoints),
            tuple(commands),
        )
        if signature == self._mission_signature:
            return
        self._mission_signature = signature

    def waypoint_reached_callback(self, msg):
        seq = int(msg.wp_seq)

        if self.active_task_id in {'', 'none'}:
            self.write_event('waypoint_reached', seq=seq)
            return

        if seq not in {self.a_seq, self.r_seq, self.release_seq, self.d_seq}:
            self.write_event(
                'waypoint_reached',
                seq=seq,
            )

        if self.release_seq is not None and seq == self.release_seq:
            gps = self._gps_sample()
            self.write_event(
                'servo_command_reached',
                seq=seq,
                release_command_seq=seq,
                gps=gps,
                r_point=dict(self.r_point) if self.r_point is not None else None,
                distance_to_r_m=self._distance_from_gps_to_r(gps),
            )

        if self.r_seq is not None and seq == self.r_seq and not self._r_reached_logged:
            self._r_reached_logged = True
            self.write_event(
                'r_reached_local_observation',
                seq=seq,
                altitude_m=self._gps_altitude(),
                relative_altitude_m=self.last_relative_alt_m,
                groundspeed_mps=self._vfr_field('groundspeed'),
                airspeed_mps=self._vfr_field('airspeed'),
                heading_deg=self._vfr_field('heading'),
                gps=self._gps_sample(),
            )
        if self.d_seq is not None and seq == self.d_seq:
            self.write_event(
                'd_reached',
                seq=seq,
                nearest_c_distance_m=self.nearest_c_distance_m,
                nearest_c_sample=self.nearest_c_sample,
            )

    def gps_callback(self, msg):
        self.last_gps = msg
        if self.target is None:
            return
        distance = self.distance_m(
            float(msg.latitude),
            float(msg.longitude),
            self.target['latitude'],
            self.target['longitude'],
        )
        if self.nearest_c_distance_m is None or distance < self.nearest_c_distance_m:
            self.nearest_c_distance_m = distance
            self.nearest_c_sample = {
                'latitude': float(msg.latitude),
                'longitude': float(msg.longitude),
                'altitude_m': float(msg.altitude),
                'distance_m': distance,
                'time': self.wall_time(),
            }

    def _maybe_log_c_distance(self, distance):
        now = self.now_sec()
        if now - self._last_c_distance_log_time < self.c_distance_log_period_s:
            return
        if (
            self._last_logged_c_distance_m is not None
            and self._last_logged_c_distance_m - distance
            < self.c_distance_improvement_step_m
        ):
            return
        self._last_c_distance_log_time = now
        self._last_logged_c_distance_m = distance
        self.write_event(
            'c_distance_update',
            distance_m=distance,
            nearest_distance_m=self.nearest_c_distance_m,
            gps=self._gps_sample(),
        )

    def relative_alt_callback(self, msg):
        self.last_relative_alt_m = float(msg.data)

    def vfr_callback(self, msg):
        self.last_vfr = msg

    def state_callback(self, msg):
        snapshot = {
            'connected': bool(msg.connected),
            'armed': bool(msg.armed),
            'mode': str(msg.mode),
            'system_status': int(msg.system_status),
        }

        previous = self.last_state
        self.last_state = snapshot

        if previous is None:
            if snapshot['connected']:
                self.write_event(
                    'fcu_connection_changed',
                    connected=True,
                )
            return

        if snapshot['connected'] != previous['connected']:
            self.write_event(
                'fcu_connection_changed',
                connected=snapshot['connected'],
            )

        if (
            snapshot['mode'] != previous['mode']
            and snapshot['mode']
        ):
            self.write_event(
                'flight_mode_changed',
                previous_mode=previous['mode'] or 'UNKNOWN',
                mode=snapshot['mode'],
            )

        if snapshot['armed'] != previous['armed']:
            self.write_event(
                'armed_state_changed',
                armed=snapshot['armed'],
            )

    def payload_status_callback(self, msg):
        if not msg.status:
            return

        if self.active_task_id in {'', 'none'}:
            return

        values = {
            item.key: item.value
            for item in msg.status[0].values
        }

        pwm_confirmed = str(
            values.get('pwm_confirmed', '')
        ).strip().lower()

        if pwm_confirmed not in {'true', '1', 'yes'}:
            return

        if self._payload_flags.get('pwm_confirmed') is True:
            return

        self._payload_flags['pwm_confirmed'] = True

        gps = self._gps_sample()
        self.write_event(
            'payload_release_confirmed',
            release_command_seq=self._coerce_value(
                values.get('release_command_seq')
            ),
            expected_pwm=self._coerce_value(
                values.get('expected_pwm')
            ),
            observed_pwm=self._coerce_value(
                values.get('observed_pwm')
            ),
            gps=gps,
            relative_altitude_m=self.last_relative_alt_m,
            groundspeed_mps=self._vfr_field('groundspeed'),
            airspeed_mps=self._vfr_field('airspeed'),
            r_point=dict(self.r_point) if self.r_point is not None else None,
            distance_to_r_m=self._distance_from_gps_to_r(gps),
        )

    def _distance_from_gps_to_r(self, gps):
        if not isinstance(gps, dict) or not isinstance(self.r_point, dict):
            return None
        try:
            return self.distance_m(
                float(gps['latitude']),
                float(gps['longitude']),
                float(self.r_point['lat']),
                float(self.r_point['lon']),
            )
        except (KeyError, TypeError, ValueError):
            return None

    def _parse_event_payload(self, payload):
        payload = str(payload).strip()
        if not payload:
            return {}
        try:
            parsed = json.loads(payload)
            if isinstance(parsed, dict):
                return parsed
        except json.JSONDecodeError:
            pass

        result = {}
        for part in payload.split():
            if '=' not in part:
                continue
            key, value = part.split('=', 1)
            result[key] = self._coerce_value(value)
        return result

    @staticmethod
    def _coerce_value(value):
        text = str(value)
        lower = text.lower()
        if lower == 'true':
            return True
        if lower == 'false':
            return False
        if lower in {'none', 'null', 'unknown'}:
            return None
        try:
            if any(mark in text for mark in ('.', 'e', 'E')):
                return float(text)
            return int(text)
        except ValueError:
            return text

    @staticmethod
    def _coerce_optional_int(value):
        if value is None:
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    def _gps_altitude(self):
        return None if self.last_gps is None else float(self.last_gps.altitude)

    def _gps_sample(self):
        if self.last_gps is None:
            return None
        return {
            'latitude': float(self.last_gps.latitude),
            'longitude': float(self.last_gps.longitude),
            'altitude_m': float(self.last_gps.altitude),
        }

    def _vfr_field(self, name):
        if self.last_vfr is None:
            return None
        value = getattr(self.last_vfr, name, None)
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return value

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        radius_m = 6371000.0
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        d_phi = math.radians(lat2 - lat1)
        d_lambda = math.radians(lon2 - lon1)
        a = (
            math.sin(d_phi / 2.0) ** 2
            + math.cos(phi1)
            * math.cos(phi2)
            * math.sin(d_lambda / 2.0) ** 2
        )
        return radius_m * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlightSummaryLoggerNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
