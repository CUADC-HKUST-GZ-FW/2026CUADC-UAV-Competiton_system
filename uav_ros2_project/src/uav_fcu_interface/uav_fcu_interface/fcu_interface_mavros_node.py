import math
import time
import copy
import json
import threading
import asyncio
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State, Waypoint, WaypointList, WaypointReached
from mavros_msgs.srv import (
    WaypointClear,
    WaypointPush,
    WaypointPull,
    WaypointSetCurrent,
)
from std_msgs.msg import String

from uav_interfaces.srv import GoToGlobal

from uav_fcu_interface.mission_verification import verify_mission_waypoints


class FcuInterfaceMavrosNode(Node):
    """Upload and monitor a verified composite AUTO mission."""

    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
    MAV_CMD_NAV_WAYPOINT = 16
    MAV_CMD_DO_SET_SERVO = 183

    HOME_SEQ = 0

    def __init__(self):
        super().__init__('fcu_interface_mavros_node')

        self.callback_group = ReentrantCallbackGroup()

        self.current_state = None
        self.current_gps = None
        self.current_raw_gps = None
        self.current_waypoints = None

        self.last_reached_seq = -1
        self.active_task_id = 'none'
        self.log_state = 'AUTO_MONITOR'
        self.mission_type = 'UNKNOWN'
        self.composite_completion_reported = False
        self.composite_final_seq = None
        self.composite_total_count = 0
        self.composite_seq_a = None
        self.composite_seq_r = None
        self.composite_seq_release = None
        self.composite_seq_d = None
        self.composite_route_points = None
        self.composite_ab_check_result = None
        # Composite A-B trajectory checker state:
        # WAIT_A -> ALIGNING -> EVALUATING -> FINALIZED
        self.composite_ab_check_state = 'WAIT_A'
        self.composite_c_confirmed = False
        self.composite_c_confirmation = 'none'
        self.composite_c_min_distance_m = float('inf')
        self._last_connected = None
        self._last_b_check_pending_log_time = 0.0
        self._gps_lock = threading.Lock()

        # General
        self.declare_parameter('dry_run_goto', True)
        self.declare_parameter('allow_mission_upload', False)
        self.declare_parameter('service_timeout_sec', 8.0)  # MAVROS 服务超时
        self.declare_parameter('service_availability_timeout_sec', 2.0)
        self.declare_parameter('mission_service_retry_count', 3)
        self.declare_parameter('mission_service_retry_delay_sec', 0.15)
        # A start_index=0 WaypointPush is a full mission replacement.  Clearing
        # first is both unnecessary and rejected by ArduPilot while an armed
        # AUTO mission is running, so keep it disabled unless explicitly needed.
        self.declare_parameter('clear_mission_before_full_push', False)

        # Composite mission splice control.
        self.declare_parameter('insert_wp_index', 5)
        self.declare_parameter('resume_wp_index', 8)

        # Temporary AUTO mission geometry.
        self.declare_parameter('a_offset_m', 160.0)
        self.declare_parameter('b_offset_m', 95.0)
        self.declare_parameter('release_offset_m', 56.0)
        self.declare_parameter('d_offset_m', 160.0)

        # Observation-only A-B trajectory check.
        self.declare_parameter('b_check_half_width_m', 12.0)
        self.declare_parameter('b_check_length_m', 60.0)
        self.declare_parameter('b_check_end_distance_from_a_m', 50.0)
        self.declare_parameter('b_check_required_samples', 3)
        self.declare_parameter('b_check_max_heading_error_deg', 20.0)
        self.declare_parameter('b_check_convergence_tolerance_m', 1.5)
        self.declare_parameter('b_check_min_converging_ratio', 0.5)
        self.declare_parameter('gps_stale_timeout_sec', 1.0)

        # 真实载荷动作默认关闭，仅双重授权时允许写入mission。
        self.declare_parameter('control_mode', 'DRY_RUN')
        self.declare_parameter('enable_real_payload_release', False)
        self.declare_parameter('servo_channel', 7)
        self.declare_parameter('safe_pwm', 1350)
        self.declare_parameter('release_pwm', 1900)

        # Mission waypoint acceptance radius.
        self.declare_parameter('a_acceptance_radius_m', 30.0)
        self.declare_parameter('b_acceptance_radius_m', 15.0)
        self.declare_parameter('c_acceptance_radius_m', 8.0)
        self.declare_parameter('d_acceptance_radius_m', 30.0)

        # Composite target-segment altitude.
        self.declare_parameter('mission_altitude_m', 15.0)

        self.dry_run_goto = bool(self.get_parameter('dry_run_goto').value)
        self.allow_mission_upload = bool(
            self.get_parameter('allow_mission_upload').value
        )
        self.service_timeout_sec = float(self.get_parameter('service_timeout_sec').value)
        self.service_availability_timeout_sec = max(
            0.1,
            float(self.get_parameter('service_availability_timeout_sec').value),
        )
        self.mission_service_retry_count = max(
            1, int(self.get_parameter('mission_service_retry_count').value)
        )
        self.mission_service_retry_delay_sec = max(
            0.0,
            float(self.get_parameter('mission_service_retry_delay_sec').value),
        )
        self.clear_mission_before_full_push = bool(
            self.get_parameter('clear_mission_before_full_push').value
        )
        # Serializes complete pull/build/push/verify transactions.  A reentrant
        # callback group otherwise permits two callers to interleave MAVROS'
        # stateful mission protocol.
        self._mission_update_lock = threading.Lock()

        self.insert_wp_index = int(self.get_parameter('insert_wp_index').value)
        self.resume_wp_index = int(self.get_parameter('resume_wp_index').value)

        self.a_offset_m = float(self.get_parameter('a_offset_m').value)
        self.b_offset_m = float(self.get_parameter('b_offset_m').value)
        self.release_offset_m = float(self.get_parameter('release_offset_m').value)
        self.d_offset_m = float(self.get_parameter('d_offset_m').value)

        self.b_check_half_width_m = float(
            self.get_parameter('b_check_half_width_m').value
        )
        self.b_check_length_m = float(
            self.get_parameter('b_check_length_m').value
        )
        self.b_check_end_distance_from_a_m = max(
            1.0,
            float(self.get_parameter('b_check_end_distance_from_a_m').value),
        )
        self.b_check_required_samples = max(
            3, int(self.get_parameter('b_check_required_samples').value)
        )
        self.b_check_max_heading_error_deg = min(
            180.0,
            max(
                0.0,
                float(self.get_parameter('b_check_max_heading_error_deg').value),
            ),
        )
        self.b_check_convergence_tolerance_m = max(
            0.0,
            float(self.get_parameter('b_check_convergence_tolerance_m').value),
        )
        self.b_check_min_converging_ratio = min(
            1.0,
            max(
                0.0,
                float(self.get_parameter('b_check_min_converging_ratio').value),
            ),
        )
        self.gps_stale_timeout_sec = float(
            self.get_parameter('gps_stale_timeout_sec').value
        )
        self.control_mode = str(self.get_parameter('control_mode').value).upper()
        self.enable_real_payload_release = bool(
            self.get_parameter('enable_real_payload_release').value
        )
        self.servo_channel = int(self.get_parameter('servo_channel').value)
        self.safe_pwm = int(self.get_parameter('safe_pwm').value)
        self.release_pwm = int(self.get_parameter('release_pwm').value)

        self.release_command_enabled, self.release_command_reason = (
            self.can_insert_release_command()
        )

        self.a_acceptance_radius_m = float(
            self.get_parameter('a_acceptance_radius_m').value
        )
        self.b_acceptance_radius_m = float(
            self.get_parameter('b_acceptance_radius_m').value
        )
        self.c_acceptance_radius_m = float(
            self.get_parameter('c_acceptance_radius_m').value
        )
        self.d_acceptance_radius_m = float(
            self.get_parameter('d_acceptance_radius_m').value
        )

        self.mission_altitude_m = float(self.get_parameter('mission_altitude_m').value)
        history_size = max(20, self.b_check_required_samples * 6)
        self.gps_history = deque(maxlen=history_size)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10,
            callback_group=self.callback_group
        )

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            sensor_qos,
            callback_group=self.callback_group
        )

        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/raw/fix',
            self.raw_gps_callback,
            sensor_qos,
            callback_group=self.callback_group
        )

        self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.waypoints_callback,
            10,
            callback_group=self.callback_group
        )

        self.create_subscription(
            WaypointReached,
            '/mavros/mission/reached',
            self.waypoint_reached_callback,
            10,
            callback_group=self.callback_group
        )

        task_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String,
            '/mission/active_task_id',
            self.task_id_callback,
            task_qos,
            callback_group=self.callback_group,
        )
        self.composite_complete_publisher = self.create_publisher(
            String,
            '/fcu/composite_mission_complete',
            10,
        )
        self.mission_summary_event_publisher = self.create_publisher(
            String,
            '/fcu/mission_summary_event',
            10,
        )

        self.mission_clear_client = self.create_client(
            WaypointClear,
            '/mavros/mission/clear',
            callback_group=self.callback_group
        )

        self.mission_push_client = self.create_client(
            WaypointPush,
            '/mavros/mission/push',
            callback_group=self.callback_group
        )

        self.mission_pull_client = self.create_client(
            WaypointPull,
            '/mavros/mission/pull',
            callback_group=self.callback_group
        )

        self.mission_set_current_client = self.create_client(
            WaypointSetCurrent,
            '/mavros/mission/set_current',
            callback_group=self.callback_group
        )

        self.goto_srv = self.create_service(
            GoToGlobal,
            '/fcu/goto_global',
            self.handle_goto_global,
            callback_group=self.callback_group
        )

        self.create_timer(
            5.0,
            self.print_status,
            callback_group=self.callback_group
        )

        self.get_logger().info(
            self._prefix('SAFETY', 'STARTUP')
            + ' configuration validated '
            f'dry_run_goto={self._bool(self.dry_run_goto)} '
            f'allow_mission_upload={self._bool(self.allow_mission_upload)} '
            f'insert_wp_index={self.insert_wp_index} '
            f'resume_wp_index={self.resume_wp_index} '
            f'a_offset_m={self.a_offset_m} '
            f'b_offset_m={self.b_offset_m} '
            f'release_offset_m={self.release_offset_m} '
            f'd_offset_m={self.d_offset_m} '
            f'a_radius={self.a_acceptance_radius_m} '
            f'b_radius={self.b_acceptance_radius_m} '
            f'c_radius={self.c_acceptance_radius_m} '
            f'd_radius={self.d_acceptance_radius_m} '
            f'mission_altitude_m={self.mission_altitude_m} '
            f'control_mode={self.control_mode} '
            f'release_command_enabled={self._bool(self.release_command_enabled)} '
            f'release_command_reason={self._reason(self.release_command_reason)}'
        )

    # -------------------------
    # ROS callbacks
    # -------------------------

    def state_callback(self, msg):
        connected = bool(msg.connected)
        if self._last_connected is None or connected != self._last_connected:
            event = 'connection changed' if connected else 'connection lost'
            self.get_logger().info(
                self._prefix('FCU', 'MONITORING')
                + f' {event} connected={self._bool(connected)}'
            )
        self._last_connected = connected
        self.current_state = msg

    def task_id_callback(self, msg):
        self.active_task_id = msg.data.strip() or 'none'

    def gps_callback(self, msg):
        self.current_gps = msg
        self._record_gps_sample(msg)
        self.update_composite_trajectory_check(msg)

    def raw_gps_callback(self, msg):
        self.current_raw_gps = msg
        if self.current_gps is None:
            self.current_gps = msg
            self._record_gps_sample(msg)
            self.update_composite_trajectory_check(msg)

    def waypoints_callback(self, msg):
        self.current_waypoints = msg

    def waypoint_reached_callback(self, msg):
        self.last_reached_seq = int(msg.wp_seq)
        self.get_logger().info(
            self._prefix('FCU')
            + f' waypoint reached seq={self.last_reached_seq} '
            f'mission_type={self.mission_type} '
            f'item={self.mission_item_name(self.last_reached_seq)}'
        )
        self.update_composite_trajectory_from_reached(self.last_reached_seq)
        self.maybe_report_composite_mission_complete()

    def clear_residual_route_checks(self, clear_route=True):
        """Reset per-task trajectory state without changing the active mission."""
        self.reset_b_check_window()
        self._last_b_check_pending_log_time = 0.0
        self.composite_ab_check_result = None
        self.composite_ab_check_state = 'WAIT_A'
        self.composite_c_confirmed = False
        self.composite_c_confirmation = 'none'
        self.composite_c_min_distance_m = float('inf')
        if clear_route:
            self.composite_route_points = None

    def finalize_composite_ab_check(self, force=False, force_reason='manual_force'):
        """Evaluate the observed A-to-B track; this check is observation-only."""
        if self.composite_ab_check_result is not None:
            return
        if self.composite_route_points is None:
            return
        if self.composite_ab_check_state != 'EVALUATING':
            return
        passed, reason, metrics = self.check_b_approach(
            self.get_best_gps(),
            self.composite_route_points['A'],
            self.composite_route_points['B'],
            self.composite_route_points['heading_deg'],
        )
        latest_along_track_m = metrics.get('latest_along_track_m', float('-inf'))
        deadline_m = metrics.get(
            'deadline_m',
            self.b_check_end_distance_from_a_m,
        )
        deadline_reached = (
            math.isfinite(latest_along_track_m)
            and latest_along_track_m >= deadline_m
        )
        if passed and not force:
            finalize_reason = 'early_pass'
        elif force:
            finalize_reason = str(force_reason)
        elif deadline_reached:
            finalize_reason = 'distance_deadline'
        else:
            if time.time() - self._last_b_check_pending_log_time > 1.0:
                self.get_logger().info(
                    self._prefix('FCU', 'EXECUTING')
                    + ' composite A-B trajectory check pending '
                    f'reason={self._reason(reason)} '
                    f'sample_count={metrics["sample_count"]} '
                    f'along_track_m={latest_along_track_m:.2f} '
                    f'deadline_m={deadline_m:.2f} '
                    'finalized=false observation_only=true'
                )
                self._last_b_check_pending_log_time = time.time()
            return
        if passed is None:
            passed = False
        self.composite_ab_check_result = {
            'passed': bool(passed),
            'reason': reason,
            'metrics': metrics,
            'finalize_reason': finalize_reason,
        }
        self.composite_ab_check_state = 'FINALIZED'
        self.publish_composite_ab_check_summary()
        level = self.get_logger().info if passed else self.get_logger().warning
        level(
            self._prefix('FCU', 'EXECUTING')
            + f' composite A-B trajectory check passed={self._bool(passed)} '
            f'reason={self._reason(reason)} '
            f'finalize_reason={self._reason(finalize_reason)} '
            f'cross_track_m={metrics["cross_track_m"]:.2f} '
            f'max_cross_track_m={metrics["max_cross_track_m"]:.2f} '
            f'heading_error_deg={metrics["heading_error_deg"]:.2f} '
            f'convergence_slope={metrics["convergence_slope_m_per_sample"]:.2f} '
            f'converging_ratio={metrics["converging_ratio"]:.2f} '
            f'forward_progress_m={metrics["forward_progress_m"]:.2f} '
            f'along_track_m={latest_along_track_m:.2f} '
            f'deadline_m={deadline_m:.2f} '
            f'sample_count={metrics["sample_count"]} observation_only=true'
        )

    def confirm_composite_c_passage(self, evidence, distance_m=None):
        if self.composite_c_confirmed:
            return
        self.composite_c_confirmed = True
        self.composite_c_confirmation = str(evidence)
        if distance_m is not None:
            self.composite_c_min_distance_m = min(
                self.composite_c_min_distance_m,
                float(distance_m),
            )
        self.publish_mission_summary_event(
            'composite_c_passage_confirmed',
            success=True,
            evidence=str(evidence),
            min_distance_m=(
                self.composite_c_min_distance_m
                if math.isfinite(self.composite_c_min_distance_m)
                else None
            ),
            acceptance_radius_m=self.c_acceptance_radius_m,
        )
        self.get_logger().info(
            self._prefix('FCU', 'EXECUTING')
            + f' composite C passage confirmed evidence={evidence} '
            f'min_distance_m={self.composite_c_min_distance_m:.2f}'
        )
        self.reset_b_check_window()
        self._last_b_check_pending_log_time = 0.0

    def update_composite_trajectory_check(self, gps):
        if (
            self.mission_type != 'COMPOSITE'
            or self.composite_route_points is None
            or gps.status.status < 0
        ):
            return
        current_seq = -1
        if self.current_waypoints is not None:
            current_seq = int(self.current_waypoints.current_seq)
        if self.composite_seq_a is None or self.composite_seq_d is None:
            return
        target_segment_active = (
            self.last_reached_seq >= self.composite_seq_a
            or self.composite_seq_a <= current_seq <= self.composite_seq_d
        )
        if not target_segment_active:
            return

        distance_to_c_m = self.distance_m(
            gps.latitude,
            gps.longitude,
            self.composite_route_points['C']['lat'],
            self.composite_route_points['C']['lon'],
        )
        self.composite_c_min_distance_m = min(
            self.composite_c_min_distance_m,
            distance_to_c_m,
        )
        if self.composite_ab_check_result is None:
            if self.composite_ab_check_state == 'ALIGNING':
                metrics = self.get_composite_ab_alignment_metrics()
                latest_along_track_m = metrics['latest_along_track_m']
                deadline_reached = (
                    math.isfinite(latest_along_track_m)
                    and latest_along_track_m >= self.b_check_end_distance_from_a_m
                )

                if deadline_reached:
                    self.composite_ab_check_result = {
                        'passed': False,
                        'reason': 'alignment_not_established_before_deadline',
                        'metrics': metrics,
                        'finalize_reason': 'distance_deadline',
                    }
                    self.composite_ab_check_state = 'FINALIZED'
                    self.publish_composite_ab_check_summary()

                    self.get_logger().warning(
                        self._prefix('FCU', 'EXECUTING')
                        + ' composite A-B trajectory check passed=false '
                        'reason=alignment_not_established_before_deadline '
                        'finalize_reason=distance_deadline '
                        f'heading_error_deg={metrics["heading_error_deg"]:.2f} '
                        f'forward_progress_m={metrics["forward_progress_m"]:.2f} '
                        f'along_track_m={latest_along_track_m:.2f} '
                        f'deadline_m={self.b_check_end_distance_from_a_m:.2f} '
                        f'sample_count={metrics["sample_count"]} '
                        'observation_only=true'
                    )

                elif metrics['aligned']:
                    self.get_logger().info(
                        self._prefix('FCU', 'EXECUTING')
                        + ' composite A-B alignment established '
                        f'sample_count={metrics["sample_count"]} '
                        f'forward_progress_m={metrics["forward_progress_m"]:.2f} '
                        f'heading_error_deg={metrics["heading_error_deg"]:.2f} '
                        f'along_track_m={latest_along_track_m:.2f} '
                        'gps_history_reset=true '
                        'state=EVALUATING observation_only=true'
                    )
                    self.reset_b_check_window()
                    self.composite_ab_check_state = 'EVALUATING'
                    self._last_b_check_pending_log_time = 0.0

                elif time.time() - self._last_b_check_pending_log_time > 1.0:
                    self.get_logger().info(
                        self._prefix('FCU', 'EXECUTING')
                        + ' composite A-B alignment pending '
                        f'sample_count={metrics["sample_count"]} '
                        f'forward_progress_m={metrics["forward_progress_m"]:.2f} '
                        f'heading_error_deg={metrics["heading_error_deg"]:.2f} '
                        f'along_track_m={latest_along_track_m:.2f} '
                        f'deadline_m={self.b_check_end_distance_from_a_m:.2f} '
                        'state=ALIGNING observation_only=true'
                    )
                    self._last_b_check_pending_log_time = time.time()

            elif self.composite_ab_check_state == 'EVALUATING':
                self.finalize_composite_ab_check()

        if distance_to_c_m <= self.c_acceptance_radius_m:
            self.confirm_composite_c_passage('gps_radius', distance_to_c_m)

    def update_composite_trajectory_from_reached(self, reached_seq):
        if self.mission_type != 'COMPOSITE':
            return

        reached_seq = int(reached_seq)

        if reached_seq == self.composite_seq_a:
            self.reset_b_check_window()
            self.composite_ab_check_result = None
            self.composite_ab_check_state = 'ALIGNING'
            self._last_b_check_pending_log_time = 0.0

            self.publish_mission_summary_event(
                'composite_a_reached',
                success=True,
                seq=self.composite_seq_a,
            )

            self.get_logger().info(
                self._prefix('FCU', 'EXECUTING')
                + ' composite A-B alignment started '
                f'a_seq={self.composite_seq_a} '
                'state=ALIGNING gps_history_reset=true '
                'observation_only=true'
            )
            return

        if reached_seq != self.composite_seq_r:
            return

        if self.composite_ab_check_result is not None:
            return

        if self.composite_ab_check_state == 'EVALUATING':
            self.finalize_composite_ab_check(
                force=True,
                force_reason='r_reached_fallback',
            )
            return

        if self.composite_ab_check_state == 'ALIGNING':
            metrics = self.get_composite_ab_alignment_metrics()
            self.composite_ab_check_result = {
                'passed': False,
                'reason': 'alignment_not_established_before_r',
                'metrics': metrics,
                'finalize_reason': 'r_reached_fallback',
            }
            self.composite_ab_check_state = 'FINALIZED'
            self.publish_composite_ab_check_summary()

            self.get_logger().warning(
                self._prefix('FCU', 'EXECUTING')
                + ' composite A-B trajectory check passed=false '
                'reason=alignment_not_established_before_r '
                'finalize_reason=r_reached_fallback '
                f'heading_error_deg={metrics["heading_error_deg"]:.2f} '
                f'forward_progress_m={metrics["forward_progress_m"]:.2f} '
                f'along_track_m={metrics["latest_along_track_m"]:.2f} '
                f'sample_count={metrics["sample_count"]} '
                'observation_only=true'
            )
            return

        if self.composite_ab_check_state == 'WAIT_A':
            self.composite_ab_check_result = {
                'passed': False,
                'reason': 'a_reached_not_observed_before_r',
                'metrics': {},
                'finalize_reason': 'r_reached_fallback',
            }
            self.composite_ab_check_state = 'FINALIZED'
            self.publish_composite_ab_check_summary()
            self.get_logger().warning(
                self._prefix('FCU', 'EXECUTING')
                + ' composite A-B trajectory check passed=false '
                'reason=a_reached_not_observed_before_r '
                'finalize_reason=r_reached_fallback '
                'observation_only=true'
            )

    def maybe_report_composite_mission_complete(self):
        if self.mission_type != 'COMPOSITE':
            return
        if self.composite_completion_reported:
            return
        if self.current_waypoints is None:
            return
        total_count = len(self.current_waypoints.waypoints)
        if total_count <= 0:
            return
        final_seq = total_count - 1
        if self.composite_final_seq is not None:
            final_seq = int(self.composite_final_seq)
        if self.last_reached_seq < final_seq:
            return

        ab_passed = bool(
            self.composite_ab_check_result
            and self.composite_ab_check_result.get('passed')
        )
        c_min_distance_text = (
            f'{self.composite_c_min_distance_m:.2f}'
            if math.isfinite(self.composite_c_min_distance_m)
            else 'unknown'
        )
        self.composite_completion_reported = True
        message = String()
        message.data = (
            f'task_id={self.active_task_id} mission_type=COMPOSITE '
            f'final_seq={final_seq} total_count={total_count} '
            f'last_reached_seq={self.last_reached_seq} '
            f'ab_track_passed={self._bool(ab_passed)} '
            f'c_confirmed={self._bool(self.composite_c_confirmed)} '
            f'c_evidence={self.composite_c_confirmation} '
            f'c_min_distance_m={c_min_distance_text}'
        )
        self.composite_complete_publisher.publish(message)
        log = (
            self.get_logger().info
            if self.composite_c_confirmed
            else self.get_logger().warning
        )
        log(
            self._prefix('MISSION', 'COMPOSITE_COMPLETE')
            + ' composite mission complete reported '
            f'task_id={self.active_task_id} final_seq={final_seq} '
            f'total_count={total_count} last_reached_seq={self.last_reached_seq} '
            f'ab_track_passed={self._bool(ab_passed)} '
            f'c_confirmed={self._bool(self.composite_c_confirmed)} '
            f'c_min_distance_m={c_min_distance_text}'
        )
        self.publish_mission_summary_event(
            'composite_mission_completed',
            success=True,
            task_id=self.active_task_id,
            final_seq=final_seq,
            total_count=total_count,
            last_reached_seq=self.last_reached_seq,
            ab_track_passed=ab_passed,
            c_confirmed=self.composite_c_confirmed,
            c_evidence=self.composite_c_confirmation,
            c_min_distance_m=(
                self.composite_c_min_distance_m
                if math.isfinite(self.composite_c_min_distance_m)
                else None
            ),
        )
        self.clear_residual_route_checks(clear_route=True)

    def publish_composite_ab_check_summary(self):
        """Publish the finalized observation-only A-B trajectory result."""
        result = self.composite_ab_check_result
        if not result:
            return

        metrics = result.get('metrics') or {}
        self.publish_mission_summary_event(
            'composite_ab_trajectory_check',
            success=bool(result.get('passed')),
            passed=bool(result.get('passed')),
            reason=str(result.get('reason', 'unknown')),
            finalize_reason=str(
                result.get('finalize_reason', 'unknown')
            ),
            cross_track_m=metrics.get('cross_track_m'),
            max_cross_track_m=metrics.get('max_cross_track_m'),
            heading_error_deg=metrics.get('heading_error_deg'),
            convergence_slope=metrics.get(
                'convergence_slope_m_per_sample'
            ),
            converging_ratio=metrics.get('converging_ratio'),
            forward_progress_m=metrics.get('forward_progress_m'),
            sample_count=metrics.get('sample_count'),
            cross_track_limit_m=self.b_check_half_width_m,
            heading_error_limit_deg=self.b_check_max_heading_error_deg,
            observation_only=True,
        )

    def publish_mission_summary_event(self, event, **fields):
        """Publish a read-only JSON event for the flight summary logger."""
        message = String()
        payload = {
            'event': event,
            'task_id': self.active_task_id,
        }
        payload.update(fields)
        try:
            message.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        except (TypeError, ValueError) as error:
            message.data = json.dumps(
                {
                    'event': event,
                    'task_id': self.active_task_id,
                    'serialization_error': repr(error),
                },
                ensure_ascii=False,
                sort_keys=True,
            )
        self.mission_summary_event_publisher.publish(message)

    def get_best_gps(self):
        if self.current_gps is not None:
            return self.current_gps
        return self.current_raw_gps

    def print_status(self):
        if self.current_state is None:
            self.get_logger().warning(
                self._prefix('STATUS', 'MONITORING')
                + ' state telemetry unavailable topic=/mavros/state'
            )
            return

        gps = self.get_best_gps()

        gps_text = 'gps_available=false'
        if gps is not None:
            gps_text = (
                f'gps_available=true gps_lat={gps.latitude:.7f} '
                f'gps_lon={gps.longitude:.7f} gps_alt_m={gps.altitude:.2f} '
                f'gps_status={gps.status.status}'
            )

        mission_text = 'current_seq=none total=none last_reached_seq=none item=none'
        if self.current_waypoints is not None:
            current_seq = int(self.current_waypoints.current_seq)
            mission_text = (
                f'current_seq={current_seq} '
                f'total={len(self.current_waypoints.waypoints)} '
                f'last_reached_seq={self.last_reached_seq} '
                f'item={self.mission_item_name(current_seq)}'
            )

        self.get_logger().info(
            self._prefix('STATUS')
            + f' connected={self._bool(self.current_state.connected)} '
            f'armed={self._bool(self.current_state.armed)} '
            f'mode={self.current_state.mode or "UNKNOWN"} {gps_text} '
            f'mission_type={self.mission_type} {mission_text}'
        )

    # -------------------------
    # Basic helpers
    # -------------------------

    @staticmethod
    def _bool(value):
        return str(bool(value)).lower()

    @staticmethod
    def _reason(value):
        return str(value).strip().lower().replace(' ', '_')

    def _prefix(self, module, state=None):
        task_id = self.active_task_id or 'none'
        return f'[{module}][task={task_id}][state={state or self.log_state}]'

    def mission_item_name(self, seq):
        if self.mission_type == 'COMPOSITE':
            return self.composite_item_name(seq)
        return 'none'

    def mavros_connected(self):
        return self.current_state is not None and self.current_state.connected

    def current_mode(self):
        if self.current_state is None:
            return ''
        return self.current_state.mode

    def has_valid_gps(self):
        gps = self.get_best_gps()
        if gps is None:
            return False
        return gps.status.status >= 0

    def _record_gps_sample(self, msg):
        if not hasattr(self, 'gps_history'):
            return
        if msg.status.status < 0:
            return
        if not math.isfinite(msg.latitude) or not math.isfinite(msg.longitude):
            return
        sample = (time.time(), float(msg.latitude), float(msg.longitude))
        with self._gps_lock:
            self.gps_history.append(sample)

    def reset_b_check_window(self):
        with self._gps_lock:
            self.gps_history.clear()

    def get_composite_ab_alignment_metrics(self):
        """Evaluate whether the aircraft has established motion toward virtual B.

        ALIGNING intentionally uses only sample sufficiency, forward progress,
        and heading error. Cross-track and convergence are reserved for the
        later EVALUATING state so the initial fixed-wing turn is not scored.
        """
        metrics = {
            'sample_count': 0,
            'forward_progress_m': 0.0,
            'latest_along_track_m': float('-inf'),
            'ab_heading_deg': float('nan'),
            'actual_track_deg': float('nan'),
            'heading_error_deg': float('nan'),
            'aligned': False,
        }

        if self.composite_route_points is None:
            return metrics

        a_point = self.composite_route_points['A']
        b_point = self.composite_route_points['B']

        with self._gps_lock:
            gps_samples = list(self.gps_history)

        required = self.b_check_required_samples
        metrics['sample_count'] = len(gps_samples)
        if len(gps_samples) < required:
            return metrics

        bx_m, by_m = self.local_xy_m(
            a_point['lat'],
            a_point['lon'],
            b_point['lat'],
            b_point['lon'],
        )
        segment_length_m = math.hypot(bx_m, by_m)
        if segment_length_m < 1.0:
            return metrics

        unit_x = bx_m / segment_length_m
        unit_y = by_m / segment_length_m
        metrics['ab_heading_deg'] = self.normalize_heading_deg(
            math.degrees(math.atan2(unit_x, unit_y))
        )

        valid_samples = []
        for sample_time, latitude, longitude in gps_samples:
            if not math.isfinite(latitude) or not math.isfinite(longitude):
                continue
            x_m, y_m = self.local_xy_m(
                a_point['lat'],
                a_point['lon'],
                latitude,
                longitude,
            )
            along_track_m = x_m * unit_x + y_m * unit_y
            valid_samples.append(
                (sample_time, latitude, longitude, x_m, y_m, along_track_m)
            )

        metrics['sample_count'] = len(valid_samples)
        if len(valid_samples) < required:
            return metrics

        samples = valid_samples[-required:]
        first = samples[0]
        latest = samples[-1]
        metrics['sample_count'] = len(samples)
        metrics['latest_along_track_m'] = latest[5]
        metrics['forward_progress_m'] = latest[5] - first[5]

        track_x_m = latest[3] - first[3]
        track_y_m = latest[4] - first[4]
        track_distance_m = math.hypot(track_x_m, track_y_m)
        if track_distance_m > 1.0:
            actual_track_deg = self.normalize_heading_deg(
                math.degrees(math.atan2(track_x_m, track_y_m))
            )
            metrics['actual_track_deg'] = actual_track_deg
            metrics['heading_error_deg'] = (
                (
                    actual_track_deg
                    - metrics['ab_heading_deg']
                    + 180.0
                )
                % 360.0
                - 180.0
            )

        metrics['aligned'] = (
            metrics['forward_progress_m'] > 1.0
            and math.isfinite(metrics['heading_error_deg'])
            and abs(metrics['heading_error_deg'])
            <= self.b_check_max_heading_error_deg
        )
        return metrics

    def wait_for_current_seq(self, target_seq, timeout_sec=5.0):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if self.current_waypoints is not None:
                if int(self.current_waypoints.current_seq) == int(target_seq):
                    return True
            time.sleep(0.1)
        return False

    # -------------------------
    # Original AUTO trigger helpers
    # -------------------------

    # -------------------------
    # Geometry helpers
    # -------------------------

    @staticmethod
    def normalize_heading_deg(heading_deg):
        return heading_deg % 360.0

    @staticmethod
    def destination_point(latitude_deg, longitude_deg, heading_deg, distance_m):
        earth_radius_m = 6371000.0

        lat1 = math.radians(latitude_deg)
        lon1 = math.radians(longitude_deg)
        bearing = math.radians(heading_deg)
        angular_distance = distance_m / earth_radius_m

        lat2 = math.asin(
            math.sin(lat1) * math.cos(angular_distance)
            + math.cos(lat1) * math.sin(angular_distance) * math.cos(bearing)
        )

        lon2 = lon1 + math.atan2(
            math.sin(bearing) * math.sin(angular_distance) * math.cos(lat1),
            math.cos(angular_distance) - math.sin(lat1) * math.sin(lat2)
        )

        lat2_deg = math.degrees(lat2)
        lon2_deg = math.degrees(lon2)
        lon2_deg = (lon2_deg + 540.0) % 360.0 - 180.0

        return lat2_deg, lon2_deg

    @staticmethod
    def distance_m(lat1, lon1, lat2, lon2):
        earth_radius_m = 6371000.0

        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        d_phi = math.radians(lat2 - lat1)
        d_lambda = math.radians(lon2 - lon1)

        a = (
            math.sin(d_phi / 2.0) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
        )

        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        return earth_radius_m * c

    @staticmethod
    def local_xy_m(origin_lat, origin_lon, latitude, longitude):
        earth_radius_m = 6371000.0
        mean_lat = math.radians((origin_lat + latitude) * 0.5)
        x_m = math.radians(longitude - origin_lon) * earth_radius_m * math.cos(mean_lat)
        y_m = math.radians(latitude - origin_lat) * earth_radius_m
        return x_m, y_m

    def check_b_approach(self, current_gps, a_point, b_point, planned_heading_deg):
        metrics = {
            'sample_count': 0,
            'cross_track_m': float('inf'),
            'max_cross_track_m': float('inf'),
            'mean_cross_track_m': float('inf'),
            'forward_progress_m': 0.0,
            'latest_along_track_m': float('-inf'),
            'window_start_m': float('nan'),
            'window_end_m': float('nan'),
            'deadline_m': self.b_check_end_distance_from_a_m,
            'planned_heading_deg': float(planned_heading_deg),
            'ab_heading_deg': float('nan'),
            'actual_track_deg': float('nan'),
            'heading_error_deg': float('nan'),
            'cross_track_delta_m': 0.0,
            'convergence_slope_m_per_sample': float('inf'),
            'converging_ratio': 0.0,
        }
        if current_gps is None or current_gps.status.status < 0:
            return False, 'gps_invalid', metrics
        with self._gps_lock:
            gps_samples = list(self.gps_history)
        if not gps_samples:
            return None, 'gps_history_empty', metrics

        now = time.time()
        latest_age_sec = now - gps_samples[-1][0]
        metrics['gps_age_sec'] = latest_age_sec
        if latest_age_sec > self.gps_stale_timeout_sec:
            return False, 'gps_stale', metrics

        bx_m, by_m = self.local_xy_m(
            a_point['lat'], a_point['lon'], b_point['lat'], b_point['lon']
        )
        segment_length_m = math.hypot(bx_m, by_m)
        if segment_length_m < 1.0:
            return False, 'a_b_segment_too_short', metrics

        unit_x = bx_m / segment_length_m
        unit_y = by_m / segment_length_m
        metrics['ab_heading_deg'] = self.normalize_heading_deg(
            math.degrees(math.atan2(unit_x, unit_y))
        )
        window_start_m = max(0.0, segment_length_m - self.b_check_length_m)
        window_end_m = min(segment_length_m, self.b_check_end_distance_from_a_m)
        metrics['window_start_m'] = window_start_m
        metrics['window_end_m'] = window_end_m
        if window_end_m <= window_start_m:
            return False, 'b_check_window_invalid', metrics
        relevant = []
        for sample_time, latitude, longitude in gps_samples:
            x_m, y_m = self.local_xy_m(
                a_point['lat'], a_point['lon'], latitude, longitude
            )
            along_track_m = x_m * unit_x + y_m * unit_y
            cross_track_m = abs(x_m * unit_y - y_m * unit_x)
            metrics['latest_along_track_m'] = along_track_m
            if window_start_m <= along_track_m <= window_end_m:
                relevant.append(
                    (sample_time, along_track_m, cross_track_m, x_m, y_m)
                )

        required = self.b_check_required_samples
        metrics['sample_count'] = len(relevant)
        if len(relevant) < required:
            return None, 'gps_history_insufficient', metrics

        samples = relevant[-required:]
        cross_tracks = [sample[2] for sample in samples]
        forward_progress_m = samples[-1][1] - samples[0][1]
        metrics['cross_track_m'] = cross_tracks[-1]
        metrics['max_cross_track_m'] = max(cross_tracks)
        metrics['mean_cross_track_m'] = sum(cross_tracks) / len(cross_tracks)
        metrics['forward_progress_m'] = forward_progress_m
        metrics['cross_track_delta_m'] = cross_tracks[0] - cross_tracks[-1]

        track_x_m = samples[-1][3] - samples[0][3]
        track_y_m = samples[-1][4] - samples[0][4]
        track_distance_m = math.hypot(track_x_m, track_y_m)
        if track_distance_m > 1.0:
            metrics['actual_track_deg'] = self.normalize_heading_deg(
                math.degrees(math.atan2(track_x_m, track_y_m))
            )
            metrics['heading_error_deg'] = (
                (metrics['actual_track_deg'] - metrics['ab_heading_deg'] + 180.0)
                % 360.0
                - 180.0
            )

        sample_indices = list(range(required))
        mean_index = sum(sample_indices) / required
        mean_cross_track_m = metrics['mean_cross_track_m']
        slope_denominator = sum(
            (index - mean_index) ** 2 for index in sample_indices
        )
        metrics['convergence_slope_m_per_sample'] = sum(
            (index - mean_index) * (cross_track - mean_cross_track_m)
            for index, cross_track in zip(sample_indices, cross_tracks)
        ) / slope_denominator
        converging_steps = sum(
            current <= previous + self.b_check_convergence_tolerance_m
            for previous, current in zip(cross_tracks, cross_tracks[1:])
        )
        metrics['converging_ratio'] = converging_steps / (required - 1)

        if forward_progress_m <= 1.0:
            return False, 'not_progressing_toward_b', metrics
        if not math.isfinite(metrics['heading_error_deg']):
            return False, 'track_heading_unavailable', metrics
        if abs(metrics['heading_error_deg']) > self.b_check_max_heading_error_deg:
            return False, 'heading_error_exceeded', metrics
        if metrics['cross_track_m'] > self.b_check_half_width_m:
            return False, 'cross_track_exceeded', metrics
        if (
            metrics['convergence_slope_m_per_sample']
            > self.b_check_convergence_tolerance_m
            or metrics['converging_ratio'] < self.b_check_min_converging_ratio
        ):
            return False, 'cross_track_not_converging', metrics
        return True, 'approach_valid', metrics

    def compute_abcdr_points(self, target_lat, target_lon, heading_deg):
        """以目标C为基准计算A、B、提前释放点R和离场点D。"""
        heading_deg = self.normalize_heading_deg(heading_deg)
        reverse_heading_deg = self.normalize_heading_deg(heading_deg + 180.0)

        a_lat, a_lon = self.destination_point(
            target_lat,
            target_lon,
            reverse_heading_deg,
            self.a_offset_m
        )

        b_lat, b_lon = self.destination_point(
            target_lat,
            target_lon,
            reverse_heading_deg,
            self.b_offset_m
        )

        r_lat, r_lon = self.destination_point(
            target_lat,
            target_lon,
            reverse_heading_deg,
            self.release_offset_m
        )

        c_lat = target_lat
        c_lon = target_lon

        d_lat, d_lon = self.destination_point(
            target_lat,
            target_lon,
            heading_deg,
            self.d_offset_m
        )

        return {
            'A': {'lat': a_lat, 'lon': a_lon},
            'B': {'lat': b_lat, 'lon': b_lon},
            'R': {'lat': r_lat, 'lon': r_lon},
            'C': {'lat': c_lat, 'lon': c_lon},
            'D': {'lat': d_lat, 'lon': d_lon},
            'heading_deg': heading_deg,
            'reverse_heading_deg': reverse_heading_deg
        }

    def can_insert_release_command(self):
        """验证控制模式、独立开关及载荷参数。"""
        if self.control_mode != 'REAL_CONTROL':
            return False, 'control_mode is not REAL_CONTROL'
        if not self.enable_real_payload_release:
            return False, 'enable_real_payload_release is false'
        if not 1 <= self.servo_channel <= 16:
            return False, f'invalid servo_channel={self.servo_channel}'
        if not 800 <= self.safe_pwm <= 2200:
            return False, f'invalid safe_pwm={self.safe_pwm}'
        if not 800 <= self.release_pwm <= 2200:
            return False, f'invalid release_pwm={self.release_pwm}'
        if self.safe_pwm == self.release_pwm:
            return False, 'safe_pwm and release_pwm must differ'
        return True, 'REAL_CONTROL and payload release safety gate enabled'

    # -------------------------
    # Mission helpers
    # -------------------------

    def make_nav_waypoint(
        self, latitude, longitude, altitude_m, acceptance_radius_m, is_current=False
    ):
        wp = Waypoint()

        wp.frame = self.MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp.command = self.MAV_CMD_NAV_WAYPOINT
        wp.is_current = bool(is_current)
        wp.autocontinue = True

        # MAV_CMD_NAV_WAYPOINT params:
        # param1: hold time in seconds
        # param2: acceptance radius in meters
        # param3: pass radius in meters
        # param4: desired yaw angle
        wp.param1 = 0.0
        wp.param2 = float(acceptance_radius_m)
        wp.param3 = 0.0
        wp.param4 = 0.0

        wp.x_lat = float(latitude)
        wp.y_long = float(longitude)
        wp.z_alt = float(altitude_m)

        return wp

    def create_release_command(self):
        """创建位于R和C之间的MAV_CMD_DO_SET_SERVO动作。"""
        wp = Waypoint()
        wp.frame = self.MAV_FRAME_GLOBAL_RELATIVE_ALT
        wp.command = self.MAV_CMD_DO_SET_SERVO
        wp.is_current = False
        wp.autocontinue = True
        wp.param1 = float(self.servo_channel)
        wp.param2 = float(self.release_pwm)
        wp.param3 = 0.0
        wp.param4 = 0.0
        wp.x_lat = 0.0
        wp.y_long = 0.0
        wp.z_alt = 0.0
        return wp

    def build_composite_mission(
        self,
        abcdr,
        original_waypoints,
        break_index,
        resume_index,
    ):
        """Build original prefix + A/R/[release]/D + configured original suffix."""
        if not original_waypoints:
            raise ValueError('Original mission is empty; cannot splice target route.')
        if break_index <= self.HOME_SEQ:
            raise ValueError(f'Invalid break_index={break_index}; HOME cannot be replaced.')
        if break_index > len(original_waypoints):
            raise ValueError(
                f'Invalid break_index={break_index}; '
                f'original_count={len(original_waypoints)}'
            )
        if resume_index < break_index:
            raise ValueError(
                f'Invalid resume_index={resume_index}; '
                f'must be >= break_index={break_index}'
            )
        if resume_index > len(original_waypoints):
            raise ValueError(
                f'Invalid resume_index={resume_index}; '
                f'original_count={len(original_waypoints)}'
            )

        self.composite_seq_a = int(break_index)
        self.composite_seq_r = self.composite_seq_a + 1
        if self.release_command_enabled:
            self.composite_seq_release = self.composite_seq_r + 1
            self.composite_seq_d = self.composite_seq_release + 1
        else:
            self.composite_seq_release = None
            self.composite_seq_d = self.composite_seq_r + 1

        altitude_m = self.mission_altitude_m
        prefix = [
            self.clone_waypoint(wp)
            for wp in original_waypoints[:break_index]
        ]
        suffix = [
            self.clone_waypoint(wp)
            for wp in original_waypoints[resume_index:]
        ]
        for wp in prefix + suffix:
            wp.is_current = False

        attack_segment = [
            self.make_nav_waypoint(
                abcdr['A']['lat'],
                abcdr['A']['lon'],
                altitude_m,
                self.a_acceptance_radius_m,
            ),
            self.make_nav_waypoint(
                abcdr['R']['lat'],
                abcdr['R']['lon'],
                altitude_m,
                self.c_acceptance_radius_m,
            ),
        ]
        if self.release_command_enabled:
            attack_segment.append(self.create_release_command())
        attack_segment.extend([
            self.make_nav_waypoint(
                abcdr['D']['lat'],
                abcdr['D']['lon'],
                altitude_m,
                self.d_acceptance_radius_m,
            ),
        ])

        composite = prefix + attack_segment + suffix
        composite[self.composite_seq_a].is_current = True
        return composite, self.composite_seq_a

    @staticmethod
    def select_composite_start_seq(
        current_seq_before_upload,
        insert_index,
        resume_index,
        a_seq,
        d_seq,
        total_count,
    ):
        """Map the original mission current seq to the composite mission seq.

        If the aircraft has not reached the insertion point yet, keep flying the
        original prefix so A/R/D are entered naturally.  ArduPilot treats
        seq0 as HOME, so normalize current seq0 to the first executable mission
        item seq1 before calling WaypointSetCurrent.  Targets received at or
        after the configured insert_index are rejected by design, so this helper
        never maps a late current_seq back to A.
        """
        current_seq = int(current_seq_before_upload)
        insert_index = int(insert_index)
        total_count = int(total_count)

        if current_seq < 0:
            raise ValueError(f'Invalid current_seq_before_upload={current_seq}.')
        if total_count <= 0:
            raise ValueError(f'Invalid composite total_count={total_count}.')

        if current_seq == 0:
            selected_seq = 1
            reason = 'normalize_seq0_to_first_executable_mission_item'
        elif current_seq < insert_index:
            selected_seq = current_seq
            reason = 'preserve_original_prefix_current_seq'
        else:
            raise ValueError(
                'Late composite insertion is not allowed. '
                f'current_seq_before_upload={current_seq}, '
                f'insert_index={insert_index}.'
            )

        if selected_seq < 0 or selected_seq >= total_count:
            raise ValueError(
                f'Composite start seq out of range. selected_seq={selected_seq}, '
                f'total_count={total_count}, '
                f'current_seq_before_upload={current_seq}.'
            )
        return selected_seq, reason

    def composite_item_name(self, seq):
        seq = int(seq)
        if self.composite_seq_a is None or self.composite_seq_d is None:
            return 'unknown'
        if seq < self.composite_seq_a:
            return f'ORIGINAL_PREFIX_{seq}'
        if seq == self.composite_seq_a:
            return 'A'
        if seq == self.composite_seq_r:
            return 'R'
        if seq == self.composite_seq_release:
            return 'DO_SET_SERVO'
        if seq == self.composite_seq_d:
            return 'D'
        if seq > self.composite_seq_d:
            return f'ORIGINAL_SUFFIX_{seq}'
        return 'unknown'

    @staticmethod
    def verify_temporary_mission(expected, actual):
        return verify_mission_waypoints(expected, actual)

    @staticmethod
    def clone_waypoint(wp):
        return copy.deepcopy(wp)

    async def call_service_async(self, client, request, timeout_sec, name):
        if not client.wait_for_service(
            timeout_sec=self.service_availability_timeout_sec
        ):
            raise RuntimeError(f'{name} service not available.')
        future = client.call_async(request)
        start_time = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start_time > timeout_sec:
                future.cancel()
                raise TimeoutError(f'{name} request timed out.')
            await asyncio.sleep(0.05)
        try:
            result = future.result()
        except Exception as error:
            raise RuntimeError(f'{name} service call failed: {error}') from error
        if result is None:
            raise RuntimeError(f'{name} returned no result.')
        return result

    async def _retry_mission_service(self, name, operation):
        """Retry a complete MAVROS mission service transaction with backoff."""
        last_error = None
        for attempt in range(1, self.mission_service_retry_count + 1):
            try:
                return await operation()
            except (RuntimeError, TimeoutError) as error:
                last_error = error
                if attempt >= self.mission_service_retry_count:
                    break
                delay = self.mission_service_retry_delay_sec * attempt
                self.get_logger().warning(
                    self._prefix('FCU')
                    + f' {name} attempt={attempt} failed '
                    f'reason={self._reason(error)} retry_in_sec={delay:.3f}'
                )
                await asyncio.sleep(delay)
        raise RuntimeError(
            f'{name} failed after {self.mission_service_retry_count} attempts: '
            f'{last_error}'
        ) from last_error

    async def clear_mission_async(self):
        if not self.allow_mission_upload:
            raise RuntimeError('Mission mutation is disabled by allow_mission_upload.')

        async def operation():
            result = await self.call_service_async(
                self.mission_clear_client,
                WaypointClear.Request(),
                self.service_timeout_sec,
                'WaypointClear',
            )
            if not result.success:
                raise RuntimeError('WaypointClear rejected.')
            return result

        return await self._retry_mission_service('WaypointClear', operation)

    async def push_mission_async(self, waypoints):
        if not self.allow_mission_upload:
            raise RuntimeError('Mission upload is disabled by allow_mission_upload.')

        async def operation():
            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = waypoints
            result = await self.call_service_async(
                self.mission_push_client,
                req,
                self.service_timeout_sec,
                'WaypointPush',
            )
            if not result.success:
                raise RuntimeError(
                    f'WaypointPush rejected. transferred={result.wp_transfered}'
                )
            if result.wp_transfered != len(waypoints):
                raise RuntimeError(
                    f'WaypointPush incomplete. transferred={result.wp_transfered}, '
                    f'expected={len(waypoints)}'
                )
            return result

        return await self._retry_mission_service('WaypointPush', operation)

    async def pull_mission_async(self):
        req = WaypointPull.Request()
        self.current_waypoints = None
        result = await self.call_service_async(
            self.mission_pull_client,
            req,
            self.service_timeout_sec,
            'WaypointPull',
        )
        if not result.success:
            raise RuntimeError('WaypointPull rejected.')
        expected_count = int(result.wp_received)
        start_time = time.time()
        while time.time() - start_time < self.service_timeout_sec:
            if self.current_waypoints is not None:
                actual_count = len(self.current_waypoints.waypoints)
                if actual_count == expected_count:
                    return self.current_waypoints
            await asyncio.sleep(0.05)
        if self.current_waypoints is None:
            raise RuntimeError(
                'WaypointPull succeeded, but /mavros/mission/waypoints was not updated.'
            )
        raise RuntimeError(
            f'WaypointPull count mismatch. expected={expected_count}, '
            f'actual={len(self.current_waypoints.waypoints)}'
        )

    async def set_current_mission_item_async(self, seq):
        if not self.allow_mission_upload:
            raise RuntimeError('Mission set-current is disabled by allow_mission_upload.')
        req = WaypointSetCurrent.Request()
        req.wp_seq = int(seq)
        result = await self.call_service_async(
            self.mission_set_current_client,
            req,
            self.service_timeout_sec,
            'WaypointSetCurrent',
        )
        if not result.success:
            raise RuntimeError(f'WaypointSetCurrent rejected for seq={seq}.')
        return result

    # -------------------------
    # Monitoring temporary C
    # -------------------------

    # -------------------------
    # Services
    # -------------------------

    def handle_goto_global(self, request, response):
        target_lat = float(request.latitude)
        target_lon = float(request.longitude)
        heading_deg = self.normalize_heading_deg(float(request.heading_deg))

        self.log_state = 'PLANNING'
        self.get_logger().info(
            self._prefix('PLAN')
            + f' composite mission planning started target_lat={target_lat:.7f} '
            f'target_lon={target_lon:.7f} heading_deg={heading_deg:.2f} '
            f'mode_before_upload={self.current_mode()}'
        )

        abcdr = self.compute_abcdr_points(target_lat, target_lon, heading_deg)

        self.get_logger().info(
            self._prefix('PLAN')
            + f' parameters altitude_m={self.mission_altitude_m:.1f} '
            f'a_offset_m={self.a_offset_m:.1f} b_offset_m={self.b_offset_m:.1f} '
            f'release_offset_m={self.release_offset_m:.1f} '
            f'd_offset_m={self.d_offset_m:.1f} '
            f'c_acceptance_radius_m={self.c_acceptance_radius_m:.1f}'
        )

        radii = {
            'A': self.a_acceptance_radius_m,
            'B': self.b_acceptance_radius_m,
            'R': self.c_acceptance_radius_m,
            'C': self.c_acceptance_radius_m,
            'D': self.d_acceptance_radius_m,
        }
        for item in ('A', 'B', 'R', 'C', 'D'):
            self.get_logger().info(
                self._prefix('PLAN')
                + f' point computed item={item} lat={abcdr[item]["lat"]:.7f} '
                f'lon={abcdr[item]["lon"]:.7f} '
                f'acceptance_radius_m={radii[item]:.1f}'
            )

        if self.release_command_enabled:
            self.get_logger().info(
                self._prefix('PLAN')
                + f' payload command planned command={self.MAV_CMD_DO_SET_SERVO} '
                f'channel={self.servo_channel} pwm={self.release_pwm} '
                'position=between_R_and_D'
            )
            order = 'original_prefix,A,R,DO_SET_SERVO,D,original_suffix'
        else:
            order = 'original_prefix,A,R,D,original_suffix'
        self.get_logger().info(
            self._prefix('PLAN')
            + f' composite mission insert_layout={order} '
            'virtual_check_points=B,C'
        )

        safety_state = 'PLANNING' if self.release_command_enabled else 'REJECTED'
        safety_event = (
            'payload command permitted'
            if self.release_command_enabled
            else 'payload command rejected'
        )
        self.get_logger().info(
            self._prefix('SAFETY', safety_state)
            + f' {safety_event} reason={self._reason(self.release_command_reason)} '
            f'channel={self.servo_channel} pwm={self.release_pwm}'
        )

        if not self.mavros_connected():
            self.get_logger().warning(
                self._prefix('SAFETY', 'REJECTED')
                + ' mission rejected reason=mavros_disconnected'
            )

            self.publish_mission_summary_event(
                'composite_mission_rejected',
                success=False,
                upload_attempted=False,
                reason='mavros_disconnected',
            )
            response.success = False
            response.message = 'MAVROS is not connected. Composite mission rejected.'
            return response

        if not self.has_valid_gps():
            self.get_logger().warning(
                self._prefix('SAFETY', 'REJECTED')
                + ' mission rejected reason=gps_invalid'
            )

            self.publish_mission_summary_event(
                'composite_mission_rejected',
                success=False,
                upload_attempted=False,
                reason='gps_invalid',
            )
            response.success = False
            response.message = 'No valid GPS data. Composite mission rejected.'
            return response

        if self.dry_run_goto:
            self.get_logger().warning(
                self._prefix('SAFETY', 'REJECTED')
                + ' mission upload rejected reason=dry_run_enabled'
            )
            self.publish_mission_summary_event(
                'composite_mission_dry_run',
                success=True,
                upload_attempted=False,
                reason='dry_run_enabled',
                target_lat=target_lat,
                target_lon=target_lon,
                heading_deg=heading_deg,
                points=abcdr,
                release_command_enabled=self.release_command_enabled,
            )
            response.success = True
            response.message = (
                'Dry-run composite AUTO mission accepted. '
                f'A=({abcdr["A"]["lat"]:.7f},{abcdr["A"]["lon"]:.7f}), '
                f'B=({abcdr["B"]["lat"]:.7f},{abcdr["B"]["lon"]:.7f}), '
                f'R=({abcdr["R"]["lat"]:.7f},{abcdr["R"]["lon"]:.7f}), '
                f'C=({abcdr["C"]["lat"]:.7f},{abcdr["C"]["lon"]:.7f}), '
                f'D=({abcdr["D"]["lat"]:.7f},{abcdr["D"]["lon"]:.7f}), '
                f'release_command_enabled={self.release_command_enabled}'
            )
            return response

        if not self.allow_mission_upload:
            self.get_logger().warning(
                self._prefix('SAFETY', 'REJECTED')
                + ' mission upload rejected reason=allow_mission_upload_false'
            )
            self.publish_mission_summary_event(
                'composite_mission_rejected',
                success=False,
                upload_attempted=False,
                reason='allow_mission_upload_false',
                target_lat=target_lat,
                target_lon=target_lon,
                heading_deg=heading_deg,
                release_command_enabled=self.release_command_enabled,
            )
            response.success = False
            response.message = 'Mission upload is disabled by allow_mission_upload.'
            return response

        mode_before_upload = str(self.current_mode() or 'UNKNOWN').strip().upper()
        if mode_before_upload != 'AUTO':
            self.get_logger().warning(
                self._prefix('SAFETY', 'REJECTED')
                + ' mission upload rejected reason=flight_mode_not_auto '
                f'current_mode={mode_before_upload} '
                'automatic_mode_change=false'
            )
            self.publish_mission_summary_event(
                'composite_mission_rejected',
                success=False,
                upload_attempted=False,
                reason='flight_mode_not_auto',
                current_mode=mode_before_upload,
            )
            response.success = False
            response.message = (
                'Aircraft must already be in AUTO. '
                'FcuInterface will not change flight mode.'
            )
            return response

        self.get_logger().info(
            self._prefix('SAFETY', 'UPLOADING')
            + f' composite mission upload permitted control_mode={self.control_mode} '
            f'dry_run_goto={self._bool(self.dry_run_goto)} '
            f'mode_before_upload={self.current_mode()}'
        )

        if not self._mission_update_lock.acquire(blocking=False):
            self.publish_mission_summary_event(
                'composite_mission_rejected',
                success=False,
                upload_attempted=False,
                reason='mission_update_already_in_progress',
            )
            response.success = False
            response.message = 'Another mission update is already in progress.'
            return response
        try:
            message = asyncio.run(self.handle_goto_global_composite_async(abcdr))
        except Exception as error:
            self.get_logger().error(
                self._prefix('FCU', 'FAILED')
                + f' composite mission update failed reason={self._reason(error)}'
            )
            self.publish_mission_summary_event(
                'composite_mission_failed',
                success=False,
                reason=str(error),
                target_lat=target_lat,
                target_lon=target_lon,
                heading_deg=heading_deg,
            )
            response.success = False
            response.message = f'Composite mission update failed: {error}'
            return response
        finally:
            self._mission_update_lock.release()

        response.success = True
        response.message = message
        return response

    async def handle_goto_global_composite_async(self, abcdr):
        self.log_state = 'COMPOSITE_PULLING'
        self.mission_type = 'ORIGINAL'
        self.get_logger().info(
            self._prefix('FCU')
            + ' mission pull requested purpose=composite_source_snapshot'
        )
        pulled = await self.pull_mission_async()
        original_waypoints = [
            self.clone_waypoint(wp)
            for wp in pulled.waypoints
        ]
        original_count = len(original_waypoints)
        if original_count == 0:
            raise RuntimeError('Original mission is empty. Composite mission aborted.')
        current_seq_before_upload = int(pulled.current_seq)
        if current_seq_before_upload < 0 or current_seq_before_upload >= original_count:
            raise RuntimeError(
                f'Invalid current_seq_before_upload={current_seq_before_upload}; '
                f'original_count={original_count}.'
            )

        break_index = int(self.insert_wp_index)
        resume_index = int(self.resume_wp_index)
        if break_index <= self.HOME_SEQ or break_index >= original_count:
            self.get_logger().error(
                self._prefix('PLAN', 'FAILED')
                + f' invalid insert_wp_index={break_index} '
                f'original_count={original_count} valid_range=1..{original_count - 1}'
            )
            raise RuntimeError(
                f'Invalid insert_wp_index={break_index}; '
                f'valid range is 1..{original_count - 1}.'
            )
        if resume_index < break_index or resume_index > original_count:
            self.get_logger().error(
                self._prefix('PLAN', 'FAILED')
                + f' invalid resume_wp_index={resume_index} '
                f'insert_wp_index={break_index} original_count={original_count} '
                f'valid_range={break_index}..{original_count}'
            )
            raise RuntimeError(
                f'Invalid resume_wp_index={resume_index}; '
                f'valid range is {break_index}..{original_count}.'
            )
        if current_seq_before_upload >= break_index:
            self.get_logger().warning(
                self._prefix('SAFETY', 'REJECTED')
                + ' composite mission rejected reason=insert_window_closed '
                f'current_seq_before_upload={current_seq_before_upload} '
                f'insert_wp_index={break_index}'
            )
            raise RuntimeError(
                'Composite mission rejected because aircraft already reached '
                'or passed insert_wp_index. '
                f'current_seq_before_upload={current_seq_before_upload}, '
                f'insert_wp_index={break_index}.'
            )

        self.get_logger().info(
            self._prefix('FCU')
            + f' mission pull completed purpose=composite_source_snapshot '
            f'original_count={original_count} insert_wp_index={break_index} '
            f'resume_wp_index={resume_index} '
            f'current_seq_before_upload={current_seq_before_upload} '
            f'mode_before_upload={self.current_mode()}'
        )

        try:
            composite_waypoints, a_seq = self.build_composite_mission(
                abcdr,
                original_waypoints,
                break_index,
                resume_index,
            )
        except (KeyError, TypeError, ValueError) as error:
            raise RuntimeError(f'Failed to build composite mission: {error}') from error

        net_count_delta = len(composite_waypoints) - original_count
        attack_segment_item_count = 4 if self.release_command_enabled else 3
        release_seq_text = (
            str(self.composite_seq_release)
            if self.release_command_enabled
            else 'disabled'
        )
        self.get_logger().info(
            self._prefix('PLAN')
            + f' composite mission generated original_count={original_count} '
            f'insert_wp_index={break_index} resume_wp_index={resume_index} '
            f'total_count={len(composite_waypoints)} a_seq={a_seq} '
            f'b_virtual=true r_seq={self.composite_seq_r} '
            f'release_seq={release_seq_text} c_virtual=true '
            f'd_seq={self.composite_seq_d} '
            f'attack_segment_item_count={attack_segment_item_count} '
            f'net_count_delta={net_count_delta}'
        )
        start_seq, start_reason = self.select_composite_start_seq(
            current_seq_before_upload,
            break_index,
            resume_index,
            a_seq,
            self.composite_seq_d,
            len(composite_waypoints),
        )
        for waypoint in composite_waypoints:
            waypoint.is_current = False
        composite_waypoints[start_seq].is_current = True
        start_point = self.composite_item_name(start_seq)
        self.get_logger().info(
            self._prefix('PLAN')
            + f' composite start selected seq={start_seq} point={start_point} '
            f'reason={start_reason} '
            f'current_seq_before_upload={current_seq_before_upload} '
            f'a_seq={a_seq}'
        )
        if self.release_command_enabled:
            self.get_logger().info(
                self._prefix('PLAN')
                + f' payload command planned seq={self.composite_seq_release} '
                f'command={self.MAV_CMD_DO_SET_SERVO} '
                f'channel={self.servo_channel} pwm={self.release_pwm} '
                'mission_type=COMPOSITE'
            )
        self.clear_residual_route_checks(clear_route=True)
        self.composite_route_points = {
            key: dict(value)
            for key, value in abcdr.items()
            if isinstance(value, dict)
        }
        self.composite_route_points['heading_deg'] = float(abcdr['heading_deg'])

        self.log_state = 'COMPOSITE_UPLOADING'
        if self.clear_mission_before_full_push:
            self.get_logger().info(
                self._prefix('FCU')
                + ' mission clear requested purpose=composite_update'
            )
            await self.clear_mission_async()
            self.get_logger().info(
                self._prefix('FCU')
                + ' mission clear completed purpose=composite_update'
            )
        else:
            self.get_logger().info(
                self._prefix('FCU')
                + ' mission clear skipped purpose=full_push_replacement'
            )

        self.get_logger().info(
            self._prefix('FCU')
            + f' composite mission push requested expected_count={len(composite_waypoints)}'
        )
        await self.push_mission_async(composite_waypoints)
        self.get_logger().info(
            self._prefix('FCU')
            + f' composite mission push completed transferred={len(composite_waypoints)} '
            f'expected_count={len(composite_waypoints)}'
        )

        self.get_logger().info(
            self._prefix('FCU')
            + ' mission pull requested purpose=composite_verification'
        )
        pulled_after_push = await self.pull_mission_async()
        actual_waypoints = list(pulled_after_push.waypoints)
        ok, reason = self.verify_temporary_mission(
            composite_waypoints,
            actual_waypoints,
        )
        if not ok:
            raise RuntimeError(f'Composite mission verification failed: {reason}')
        self.composite_completion_reported = False
        self.composite_total_count = len(composite_waypoints)
        self.composite_final_seq = self.composite_seq_d
        self.mission_type = 'COMPOSITE'
        self.publish_mission_summary_event(
            'composite_mission_uploaded',
            success=True,
            upload_attempted=True,
            verified=True,
            original_count=original_count,
            total_count=len(composite_waypoints),
            insert_wp_index=break_index,
            resume_wp_index=resume_index,
            current_seq_before_upload=current_seq_before_upload,
            start_seq=start_seq,
            start_point=start_point,
            start_reason=start_reason,
            a_seq=a_seq,
            b_virtual=True,
            r_seq=self.composite_seq_r,
            release_seq=self.composite_seq_release,
            release_command_enabled=self.release_command_enabled,
            c_virtual=True,
            d_seq=self.composite_seq_d,
            target_lat=abcdr['C']['lat'],
            target_lon=abcdr['C']['lon'],
            heading_deg=abcdr['heading_deg'],
            points={
                key: dict(abcdr[key])
                for key in ('A', 'B', 'R', 'C', 'D')
            },
        )
        self.get_logger().info(
            self._prefix('FCU')
            + f' composite mission pull-back verified count={len(composite_waypoints)}'
        )
        self.get_logger().info(
            self._prefix('FULLCHAIN', 'VERIFIED')
            + ' stage=push_verified '
            f'total_count={len(composite_waypoints)} start_seq={start_seq} '
            f'target_lat={abcdr["C"]["lat"]:.8f} '
            f'target_lon={abcdr["C"]["lon"]:.8f} '
            f'release_command_enabled={self._bool(self.release_command_enabled)}'
        )

        mode_before_set_current = str(
            self.current_mode() or 'UNKNOWN'
        ).strip().upper()
        if mode_before_set_current != 'AUTO':
            self.log_state = 'COMPOSITE_MODE_CHANGED'
            raise RuntimeError(
                'Composite mission uploaded but flight mode changed before '
                f'set-current; expected=AUTO actual={mode_before_set_current}; '
                'no automatic mode change or LOITER rollback was attempted; '
                'uploaded mission remains on the FCU.'
            )

        self.log_state = 'COMPOSITE_STARTING'
        await self.set_current_mission_item_async(start_seq)
        self.get_logger().info(
            self._prefix('FCU')
            + f' mission current set seq={start_seq} point={start_point} '
            f'mission_type=COMPOSITE '
            f'mode_after_set_current={self.current_mode()}'
        )
        if not self.wait_for_current_seq(start_seq, timeout_sec=2.0):
            raise RuntimeError(
                f'Composite mission current_seq confirmation timed out. '
                f'expected={start_seq}'
            )
        mode_after_set_current = str(
            self.current_mode() or 'UNKNOWN'
        ).strip().upper()
        if mode_after_set_current != 'AUTO':
            self.log_state = 'COMPOSITE_MODE_CHANGED'
            raise RuntimeError(
                'Composite mission uploaded and current item set, but flight '
                f'mode changed; expected=AUTO actual={mode_after_set_current}; '
                'no automatic mode change or LOITER rollback was attempted; '
                'uploaded mission remains on the FCU.'
            )
        self.last_reached_seq = -1
        self.composite_completion_reported = False
        self.log_state = 'EXECUTING'
        self.get_logger().info(
            self._prefix('FCU')
            + f' mission current confirmed seq={start_seq} point={start_point} '
            f'mission_type=COMPOSITE mode_confirmed={self.current_mode()}'
        )
        self.get_logger().info(
            self._prefix('FULLCHAIN', 'EXECUTING')
            + ' stage=set_current_confirmed '
            f'seq={start_seq} point={start_point} '
            f'mode={self.current_mode()} mission_type=COMPOSITE'
        )

        return (
            f'Composite mission uploaded; AUTO remained confirmed without '
            f'an automatic mode change. '
            f'insert_wp_index={break_index}, resume_wp_index={resume_index}, '
            f'a_seq={a_seq}, start_seq={start_seq}, '
            f'original_count={original_count}, total_count={len(composite_waypoints)}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = FcuInterfaceMavrosNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
