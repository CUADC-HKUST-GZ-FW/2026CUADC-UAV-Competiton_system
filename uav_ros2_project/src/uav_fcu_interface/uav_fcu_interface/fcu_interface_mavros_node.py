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
from mavros_msgs.msg import (
    State, VfrHud, Waypoint, WaypointList, WaypointReached,
)
from mavros_msgs.srv import (
    WaypointClear,
    WaypointPush,
    WaypointPull,
    WaypointSetCurrent,
)
from std_msgs.msg import Float64, String

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
        self.current_vfr_hud = None
        self.current_rel_alt_m = None
        self.current_rel_alt_timestamp_monotonic = None
        self.current_waypoints = None
        self._waypoint_list_generation = 0

        self.last_reached_seq = -1
        self.active_task_id = 'none'
        self.log_state = 'AUTO_MONITOR'
        self.mission_type = 'UNKNOWN'
        self.composite_completion_reported = False
        self.composite_final_seq = None
        self.composite_total_count = 0
        self.composite_seq_a = None
        self.composite_seq_u = None
        self.composite_seq_r = None
        self.composite_seq_release = None
        self.composite_seq_d = None
        self.composite_route_points = None
        self.dynamic_indices = {}
        self.composite_expected_waypoints = None
        self._safe_composite_mission = None
        self.composite_safe_r_waypoint = None
        self._last_mission_current_seq = None
        self._b_previous_signed_m = None
        self._b_crossing_triggered = False
        self._dynamic_update_started = False
        self._dynamic_update_state = 'IDLE'
        self._dynamic_update_cancelled = False
        self._dynamic_worker_cancel_reason = None
        self._dynamic_metrics = {}
        self._second_full_push_attempted = False
        self.b_frozen_snapshot = None
        self._dynamic_update_verified = False
        self._dynamic_update_verified_monotonic = None
        self._dynamic_update_verified_gps = None
        self._dynamic_update_result = 'NOT_OBSERVED'
        self._r_active_before_verify_reported = False
        self._dynamic_prediction_commit = None
        self._dynamic_prediction_shadow = None
        self._dynamic_state_lock = threading.Lock()
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
        self._live_state_lock = threading.Lock()
        self.live_vehicle_state = {
            'gps': None,
            'vfr_hud': None,
            'rel_alt_m': None,
            'gps_timestamp_monotonic': None,
            'vfr_timestamp_monotonic': None,
            'rel_alt_timestamp_monotonic': None,
        }

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
        self.declare_parameter('b_offset_m', 110.0)
        # TEMPORARY TEST VALUE. Final B-U and U-R spacing must be selected from
        # measured update latency and commit-margin data.
        self.declare_parameter('u_offset_m', 80.0)
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
        self.declare_parameter('u_acceptance_radius_m', 15.0)
        self.declare_parameter('c_acceptance_radius_m', 8.0)
        self.declare_parameter('d_acceptance_radius_m', 30.0)

        # Composite target-segment altitude.
        self.declare_parameter('mission_altitude_m', 15.0)

        # Dynamic-R release point. TEST mode retains the old fixed-offset path;
        # normal mode uses the AB multi-sample predictor below.
        self.declare_parameter('dynamic_r_enabled', True)
        self.declare_parameter('dynamic_r_test_mode', False)
        self.declare_parameter('dynamic_r_test_offset_m', 50.0)
        self.declare_parameter('dynamic_r_prediction_window_sec', 1.2)
        self.declare_parameter('dynamic_r_min_prediction_samples', 5)
        self.declare_parameter('dynamic_r_min_prediction_span_sec', 0.4)
        self.declare_parameter('dynamic_r_vz_fit_max_rmse_mps', 0.8)
        self.declare_parameter('dynamic_r_max_abs_vertical_accel_mps2', 3.0)
        self.declare_parameter('dynamic_r_min_rc_m', 20.0)
        self.declare_parameter('dynamic_r_min_u_r_distance_m', 5.0)
        self.declare_parameter('dynamic_r_max_iterations', 4)
        self.declare_parameter('dynamic_r_convergence_m', 0.5)
        # Reserved for later release-mechanism calibration. Keep 0.0 now.
        self.declare_parameter('dynamic_r_release_delay_sec', 0.0)
        self.declare_parameter('dynamic_r_prediction_shadow_mode', False)
        # Software deadline for the second full-mission update, in addition to R active/
        # reached. In-flight services may finish; read-only reconciliation is
        # still required after an error, but no late success or retry is allowed.
        self.declare_parameter('dynamic_r_update_timeout_sec', 6.0)
        self.declare_parameter('aburcd_update_metrics_enabled', True)
        self.declare_parameter('target_system_id', 1)
        self.declare_parameter('target_component_id', 1)

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
        self.u_offset_m = float(self.get_parameter('u_offset_m').value)
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
        self.u_acceptance_radius_m = float(
            self.get_parameter('u_acceptance_radius_m').value
        )
        self.c_acceptance_radius_m = float(
            self.get_parameter('c_acceptance_radius_m').value
        )
        self.d_acceptance_radius_m = float(
            self.get_parameter('d_acceptance_radius_m').value
        )

        self.mission_altitude_m = float(self.get_parameter('mission_altitude_m').value)
        self.dynamic_r_enabled = bool(
            self.get_parameter('dynamic_r_enabled').value
        )
        self.dynamic_r_test_mode = bool(
            self.get_parameter('dynamic_r_test_mode').value
        )
        self.dynamic_r_test_offset_m = float(
            self.get_parameter('dynamic_r_test_offset_m').value
        )
        self.dynamic_r_prediction_window_sec = max(
            0.2,
            float(self.get_parameter('dynamic_r_prediction_window_sec').value),
        )
        self.dynamic_r_min_prediction_samples = max(
            3, int(self.get_parameter('dynamic_r_min_prediction_samples').value)
        )
        self.dynamic_r_min_prediction_span_sec = max(
            0.1,
            float(self.get_parameter('dynamic_r_min_prediction_span_sec').value),
        )
        self.dynamic_r_vz_fit_max_rmse_mps = max(
            0.0,
            float(self.get_parameter('dynamic_r_vz_fit_max_rmse_mps').value),
        )
        self.dynamic_r_max_abs_vertical_accel_mps2 = max(
            0.0,
            float(
                self.get_parameter(
                    'dynamic_r_max_abs_vertical_accel_mps2'
                ).value
            ),
        )
        self.dynamic_r_min_rc_m = max(
            0.0, float(self.get_parameter('dynamic_r_min_rc_m').value)
        )
        self.dynamic_r_min_u_r_distance_m = max(
            0.0,
            float(self.get_parameter('dynamic_r_min_u_r_distance_m').value),
        )
        self.dynamic_r_max_iterations = max(
            1, int(self.get_parameter('dynamic_r_max_iterations').value)
        )
        self.dynamic_r_convergence_m = max(
            0.01, float(self.get_parameter('dynamic_r_convergence_m').value)
        )
        self.dynamic_r_release_delay_sec = max(
            0.0, float(self.get_parameter('dynamic_r_release_delay_sec').value)
        )
        self.dynamic_r_prediction_shadow_mode = bool(
            self.get_parameter('dynamic_r_prediction_shadow_mode').value
        )
        self.dynamic_r_update_timeout_sec = max(
            0.1,
            float(self.get_parameter('dynamic_r_update_timeout_sec').value),
        )
        self.aburcd_update_metrics_enabled = bool(
            self.get_parameter('aburcd_update_metrics_enabled').value
        )
        self.target_system_id = int(self.get_parameter('target_system_id').value)
        self.target_component_id = int(
            self.get_parameter('target_component_id').value
        )
        if not (
            self.a_offset_m > self.b_offset_m > self.u_offset_m
            > self.release_offset_m > 0.0
        ):
            raise ValueError(
                'ABURCD offsets must satisfy '
                'a_offset_m > b_offset_m > u_offset_m > release_offset_m > 0'
            )
        history_size = max(
            60,
            self.b_check_required_samples * 6,
            self.dynamic_r_min_prediction_samples * 8,
        )
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
            VfrHud,
            '/mavros/vfr_hud',
            self.vfr_hud_callback,
            sensor_qos,
            callback_group=self.callback_group,
        )

        self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.rel_alt_callback,
            sensor_qos,
            callback_group=self.callback_group,
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
        self.create_subscription(
            String,
            '/mission/safety_state',
            self.mission_safety_state_callback,
            10,
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
            f'u_offset_m={self.u_offset_m} '
            f'release_offset_m={self.release_offset_m} '
            f'd_offset_m={self.d_offset_m} '
            f'a_radius={self.a_acceptance_radius_m} '
            f'b_radius={self.b_acceptance_radius_m} '
            f'u_radius={self.u_acceptance_radius_m} '
            f'c_radius={self.c_acceptance_radius_m} '
            f'd_radius={self.d_acceptance_radius_m} '
            f'mission_altitude_m={self.mission_altitude_m} '
            f'control_mode={self.control_mode} '
            f'release_command_enabled={self._bool(self.release_command_enabled)} '
            'release_command_reason='
            f'{self._reason(self.release_command_reason)} '
            f'dynamic_r_enabled={self._bool(self.dynamic_r_enabled)} '
            'dynamic_r_update_timeout_sec='
            f'{self.dynamic_r_update_timeout_sec:.3f} '
            f'dynamic_r_test_mode={self._bool(self.dynamic_r_test_mode)} '
            'dynamic_r_prediction_window_sec='
            f'{self.dynamic_r_prediction_window_sec:.2f} '
            'dynamic_r_min_prediction_samples='
            f'{self.dynamic_r_min_prediction_samples} '
            'dynamic_r_release_delay_sec='
            f'{self.dynamic_r_release_delay_sec:.3f} '
            'dynamic_r_prediction_shadow_mode='
            f'{self._bool(self.dynamic_r_prediction_shadow_mode)} '
            'aburcd_update_metrics_enabled='
            f'{self._bool(self.aburcd_update_metrics_enabled)}'
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
        previous = self.active_task_id
        self.active_task_id = msg.data.strip() or 'none'
        if previous not in {'', 'none'} and self.active_task_id in {'', 'none'}:
            self._cancel_dynamic_update('task_cleared')

    def mission_safety_state_callback(self, msg):
        state = str(msg.data).strip().upper()
        if state in {'MISSION_COMPLETE', 'SAFE'}:
            self._cancel_dynamic_update(state.lower())

    def gps_callback(self, msg):
        self.current_gps = msg
        self._update_live_gps(msg)
        self._record_gps_sample(msg)
        self.update_composite_trajectory_check(msg)
        self._maybe_detect_virtual_b_crossing(msg)

    def raw_gps_callback(self, msg):
        self.current_raw_gps = msg
        if self.current_gps is None:
            self.current_gps = msg
            self._update_live_gps(msg)
            self._record_gps_sample(msg)
            self.update_composite_trajectory_check(msg)
            self._maybe_detect_virtual_b_crossing(msg)

    def vfr_hud_callback(self, msg):
        self.current_vfr_hud = msg
        with self._live_state_lock:
            self.live_vehicle_state['vfr_hud'] = {
                'ground_speed_mps': float(msg.groundspeed),
                'vertical_speed_mps': float(msg.climb),
                'heading_deg': float(msg.heading),
            }
            self.live_vehicle_state['vfr_timestamp_monotonic'] = time.monotonic()

    def rel_alt_callback(self, msg):
        value = float(msg.data)
        if not math.isfinite(value):
            return
        now = time.monotonic()
        self.current_rel_alt_m = value
        self.current_rel_alt_timestamp_monotonic = now
        with self._live_state_lock:
            self.live_vehicle_state['rel_alt_m'] = value
            self.live_vehicle_state['rel_alt_timestamp_monotonic'] = now

    def _update_live_gps(self, msg):
        with self._live_state_lock:
            self.live_vehicle_state['gps'] = {
                'lat': float(msg.latitude),
                'lon': float(msg.longitude),
                'altitude_m': float(msg.altitude),
            }
            self.live_vehicle_state['gps_timestamp_monotonic'] = time.monotonic()

    def waypoints_callback(self, msg):
        current_seq = int(msg.current_seq)
        late_before_verified = False
        verified_at = None
        verified_gps = None
        with self._dynamic_state_lock:
            # Update the live cache and adjudicate R-active vs. verified under
            # one lock so the callback and verification worker are exclusive.
            self.current_waypoints = msg
            self._waypoint_list_generation += 1
            if current_seq == self._last_mission_current_seq:
                return
            self._last_mission_current_seq = current_seq
            if (
                self.mission_type == 'COMPOSITE'
                and self.composite_seq_r is not None
                and current_seq >= self.composite_seq_r
                and self._dynamic_update_started
                and not self._dynamic_update_verified
            ):
                self._dynamic_update_result = 'UPDATE_TOO_LATE'
                self._dynamic_update_state = 'TOO_LATE'
                self._dynamic_update_cancelled = True
                self._dynamic_worker_cancel_reason = 'mission_current_r'
                if not self._r_active_before_verify_reported:
                    self._r_active_before_verify_reported = True
                    late_before_verified = True
            verified_at = self._dynamic_update_verified_monotonic
            verified_gps = self._dynamic_update_verified_gps

        if self.mission_type != 'COMPOSITE':
            return
        item = self.composite_item_name(current_seq)
        if item in {'A', 'U', 'R'}:
            gps = self._gps_snapshot_fields()
            if item == 'R' and verified_at is not None:
                gps['r_commit_margin_sec'] = max(
                    0.0,
                    time.monotonic() - verified_at,
                )
                if verified_gps is not None:
                    gps['r_commit_margin_m'] = self.distance_m(
                        verified_gps['lat'],
                        verified_gps['lon'],
                        self.composite_route_points['R']['lat'],
                        self.composite_route_points['R']['lon'],
                    )
            self._publish_aburcd_event(
                f'mission_current_{item.lower()}',
                current_seq=current_seq,
                reached_seq=None,
                **gps,
            )
        if late_before_verified:
            self._publish_aburcd_event(
                'error_r_active_before_update_verified',
                current_seq=current_seq,
                reached_seq=None,
                failure_reason=(
                    'R became current before pull-back verification'
                ),
                **self._gps_snapshot_fields(),
            )

    def waypoint_reached_callback(self, msg):
        reached_seq = int(msg.wp_seq)
        late_before_verified = False
        with self._dynamic_state_lock:
            self.last_reached_seq = max(self.last_reached_seq, reached_seq)
            if (
                self._dynamic_update_started
                and self.composite_seq_r is not None
                and reached_seq >= self.composite_seq_r
                and not self._dynamic_update_verified
            ):
                self._dynamic_update_result = 'UPDATE_TOO_LATE'
                self._dynamic_update_state = 'TOO_LATE'
                self._dynamic_update_cancelled = True
                self._dynamic_worker_cancel_reason = 'r_reached'
                if not self._r_active_before_verify_reported:
                    self._r_active_before_verify_reported = True
                    late_before_verified = True
            elif (
                self.composite_seq_d is not None
                and reached_seq >= self.composite_seq_d
            ):
                self._dynamic_update_cancelled = True
                self._dynamic_worker_cancel_reason = 'd_reached'
                if self._dynamic_update_state not in {'VERIFIED', 'FAILED', 'TOO_LATE'}:
                    self._dynamic_update_state = 'CANCELLED'
            if (
                self.composite_seq_r is not None
                and reached_seq >= self.composite_seq_r
            ):
                self._dynamic_update_cancelled = True
                if self._dynamic_worker_cancel_reason is None:
                    self._dynamic_worker_cancel_reason = 'r_reached'
        self.get_logger().info(
            self._prefix('FCU')
            + f' waypoint reached seq={self.last_reached_seq} '
            f'mission_type={self.mission_type} '
            f'item={self.mission_item_name(self.last_reached_seq)}'
        )
        self.update_composite_trajectory_from_reached(self.last_reached_seq)
        self._record_aburcd_reached(self.last_reached_seq)
        if late_before_verified:
            self._publish_aburcd_event(
                'error_r_active_before_update_verified',
                current_seq=self._current_mission_seq(),
                reached_seq=reached_seq,
                failure_reason='R reached before dynamic update verification',
                deadline_current_seq=self._current_mission_seq(),
                deadline_last_reached_seq=self.last_reached_seq,
                **self._gps_snapshot_fields(),
            )
        self.maybe_report_composite_mission_complete()

    def _current_mission_seq(self):
        if self.current_waypoints is None:
            return None
        return int(self.current_waypoints.current_seq)

    def _dynamic_deadline_evidence_locked(self):
        current_seq = self._current_mission_seq()
        r_seq = self.dynamic_indices.get('R')
        reached_seq = self.last_reached_seq
        if r_seq is not None and reached_seq >= int(r_seq):
            return 'r_reached', current_seq, reached_seq
        if r_seq is not None and current_seq is not None and current_seq >= int(r_seq):
            return 'mission_current_r', current_seq, reached_seq
        if self._dynamic_update_cancelled:
            return (
                self._dynamic_worker_cancel_reason or 'cancelled',
                current_seq,
                reached_seq,
            )
        return None, current_seq, reached_seq

    def _dynamic_abort_reason(self, started_monotonic):
        with self._dynamic_state_lock:
            return self._dynamic_abort_reason_locked(started_monotonic)

    def _dynamic_abort_reason_locked(self, started_monotonic):
        """Check deadline/state in the same lock scope as progress telemetry."""
        reason, current_seq, reached_seq = self._dynamic_deadline_evidence_locked()
        if reason:
            if reason in {
                'r_reached', 'mission_current_r'
            }:
                self._dynamic_update_result = 'UPDATE_TOO_LATE'
                self._dynamic_update_state = 'TOO_LATE'
                self._dynamic_update_cancelled = True
                self._dynamic_worker_cancel_reason = reason
            return (
                f'{reason};deadline_current_seq={current_seq};'
                f'deadline_last_reached_seq={reached_seq}'
            )
        if self._dynamic_update_timed_out(started_monotonic):
            self._dynamic_update_state = 'FAILED'
            self._dynamic_update_result = 'UPDATE_FAILED'
            self._dynamic_update_cancelled = True
            self._dynamic_worker_cancel_reason = 'dynamic_update_timeout'
            return 'DYNAMIC_UPDATE_TIMEOUT'
        return None

    def _require_dynamic_operation_allowed(self, started_monotonic, operation):
        reason = self._dynamic_abort_reason(started_monotonic)
        if reason:
            raise DynamicUpdateAborted(f'{operation}:{reason}')

    def _cancel_dynamic_update(self, reason):
        publish = False
        with self._dynamic_state_lock:
            if self._dynamic_update_cancelled:
                return
            self._dynamic_update_cancelled = True
            self._dynamic_worker_cancel_reason = str(reason)
            if self._dynamic_update_started and not self._dynamic_update_verified:
                if self._dynamic_update_state not in {'FAILED', 'TOO_LATE'}:
                    self._dynamic_update_state = 'CANCELLED'
                publish = True
        if publish:
            self._publish_aburcd_event(
                'dynamic_update_cancelled',
                dynamic_worker_cancel_reason=str(reason),
                deadline_current_seq=self._current_mission_seq(),
                deadline_last_reached_seq=self.last_reached_seq,
                **self._gps_snapshot_fields(),
            )

    def _latch_dynamic_update_too_late(self, event, failure_reason, **fields):
        """Latch a too-late result once, using the live current-seq cache."""
        with self._dynamic_state_lock:
            live_current_seq = self._current_mission_seq()
            if self._dynamic_update_verified:
                return False
            if (
                self._dynamic_update_result == 'UPDATE_TOO_LATE'
                or self._r_active_before_verify_reported
            ):
                return False
            self._dynamic_update_result = 'UPDATE_TOO_LATE'
            self._dynamic_update_state = 'TOO_LATE'
            self._dynamic_update_cancelled = True
            self._dynamic_worker_cancel_reason = str(failure_reason)
            self._r_active_before_verify_reported = True
            self._publish_aburcd_event(
                event,
                current_seq=live_current_seq,
                reached_seq=None,
                failure_reason=failure_reason,
                **fields,
            )
            return True

    def _set_dynamic_update_result(self, result):
        """Set a non-success terminal result without overriding too-late."""
        with self._dynamic_state_lock:
            if self._dynamic_update_result == 'UPDATE_TOO_LATE':
                return False
            self._dynamic_update_result = str(result)
            if result in {'UPDATE_FAILED', 'R_SAFE_FALLBACK'}:
                self._dynamic_update_state = 'FAILED'
            return True

    def _gps_snapshot_fields(self, gps=None):
        fields = {
            'lat': None,
            'lon': None,
            'altitude_m': None,
            'relative_altitude_m': None,
            'relative_altitude_age_sec': None,
            'height_source': None,
            'ground_speed_mps': None,
            'vertical_speed_mps': None,
            'heading_deg': None,
            'distance_to_u_m': None,
            'distance_to_r_m': None,
        }
        live_vfr = None
        live_rel_alt_m = None
        live_rel_alt_timestamp = None
        if hasattr(self, 'live_vehicle_state'):
            with self._live_state_lock:
                live_rel_alt_m = self.live_vehicle_state.get('rel_alt_m')
                live_rel_alt_timestamp = self.live_vehicle_state.get(
                    'rel_alt_timestamp_monotonic'
                )
        if gps is None and hasattr(self, 'live_vehicle_state'):
            with self._live_state_lock:
                live_gps = self.live_vehicle_state.get('gps')
                live_vfr = self.live_vehicle_state.get('vfr_hud')
                live_gps = dict(live_gps) if live_gps is not None else None
                live_vfr = dict(live_vfr) if live_vfr is not None else None
            if live_gps is not None:
                fields.update(live_gps)
        else:
            gps = gps or self.get_best_gps()
        if gps is not None:
            fields.update({
                'lat': float(gps.latitude),
                'lon': float(gps.longitude),
                'altitude_m': float(gps.altitude),
            })
        if live_rel_alt_m is not None and math.isfinite(float(live_rel_alt_m)):
            fields['relative_altitude_m'] = float(live_rel_alt_m)
            fields['height_source'] = 'rel_alt'
            if live_rel_alt_timestamp is not None:
                fields['relative_altitude_age_sec'] = max(
                    0.0, time.monotonic() - float(live_rel_alt_timestamp)
                )
        if fields['lat'] is not None and self.composite_route_points is not None:
            fields['distance_to_u_m'] = self.distance_m(
                fields['lat'],
                fields['lon'],
                self.composite_route_points['U']['lat'],
                self.composite_route_points['U']['lon'],
            )
            fields['distance_to_r_m'] = self.distance_m(
                fields['lat'],
                fields['lon'],
                self.composite_route_points['R']['lat'],
                self.composite_route_points['R']['lon'],
            )
        if live_vfr is not None:
            fields.update(live_vfr)
        elif self.current_vfr_hud is not None:
            fields.update({
                'ground_speed_mps': float(self.current_vfr_hud.groundspeed),
                'vertical_speed_mps': float(self.current_vfr_hud.climb),
                'heading_deg': float(self.current_vfr_hud.heading),
            })
        return fields

    def _publish_aburcd_event(self, event, **fields):
        fields.setdefault('current_seq', self._current_mission_seq())
        fields.setdefault('reached_seq', None)
        fields.setdefault('a_seq', self.dynamic_indices.get('A'))
        fields.setdefault('u_seq', self.dynamic_indices.get('U'))
        fields.setdefault('r_seq', self.dynamic_indices.get('R'))
        fields.setdefault('release_seq', self.dynamic_indices.get('RELEASE'))
        fields.setdefault('d_seq', self.dynamic_indices.get('D'))
        self.publish_mission_summary_event(event, **fields)
        rendered = ' '.join(
            f'{key.upper() if key in {"r_source", "mission_state"} else key}='
            f'{self._reason(value)}'
            for key, value in fields.items()
            if value is not None
        )
        self.get_logger().info(
            f'[ABURCD] {event.upper()} {rendered}'.rstrip()
        )

    def _record_aburcd_reached(self, reached_seq):
        if self.mission_type != 'COMPOSITE':
            return
        item = self.composite_item_name(reached_seq)
        if item not in {'A', 'U', 'R'}:
            return
        fields = self._gps_snapshot_fields()
        self._publish_aburcd_event(
            f'{item.lower()}_reached',
            current_seq=self._current_mission_seq(),
            reached_seq=int(reached_seq),
            **fields,
        )
        if item == 'R':
            prediction = (
                self._dynamic_prediction_commit
                or self._dynamic_prediction_shadow
            )
            if prediction:
                actual_heading = fields.get('heading_deg')
                actual_ground_speed = fields.get('ground_speed_mps')
                actual_forward_speed = None
                if actual_heading is not None and actual_ground_speed is not None:
                    heading_error_deg = self._signed_heading_error_deg(
                        actual_heading,
                        self.composite_route_points['heading_deg'],
                    )
                    actual_forward_speed = float(actual_ground_speed) * math.cos(
                        math.radians(heading_error_deg)
                    )
                actual_vz = fields.get('vertical_speed_mps')
                actual_height = fields.get('relative_altitude_m')
                predicted_forward = prediction.get('predicted_v_forward_r_mps')
                predicted_vz = prediction.get('predicted_vz_r_mps')
                predicted_height = prediction.get('predicted_height_r_m')
                self._publish_aburcd_event(
                    'release_state_actual',
                    current_seq=self._current_mission_seq(),
                    reached_seq=int(reached_seq),
                    prediction_source=(
                        'dynamic_commit'
                        if self._dynamic_prediction_commit is not None
                        else 'shadow'
                    ),
                    predicted_forward_speed_r_mps=predicted_forward,
                    actual_forward_speed_r_mps=actual_forward_speed,
                    forward_speed_prediction_error_mps=(
                        actual_forward_speed - predicted_forward
                        if actual_forward_speed is not None
                        and predicted_forward is not None else None
                    ),
                    predicted_vertical_speed_r_mps=predicted_vz,
                    actual_vertical_speed_r_mps=actual_vz,
                    vertical_speed_prediction_error_mps=(
                        float(actual_vz) - float(predicted_vz)
                        if actual_vz is not None and predicted_vz is not None
                        else None
                    ),
                    predicted_height_r_m=predicted_height,
                    actual_height_r_m=actual_height,
                    height_prediction_error_m=(
                        float(actual_height) - float(predicted_height)
                        if actual_height is not None and predicted_height is not None
                        else None
                    ),
                    height_source=fields.get('height_source'),
                )

    def _virtual_b_signed_distance_m(self, gps):
        points = self.composite_route_points
        if points is None:
            return None
        bx, by = self.local_xy_m(
            points['A']['lat'], points['A']['lon'],
            points['B']['lat'], points['B']['lon'],
        )
        px, py = self.local_xy_m(
            points['A']['lat'], points['A']['lon'],
            gps.latitude, gps.longitude,
        )
        b_along = math.hypot(bx, by)
        if b_along < 1.0:
            return None
        return (px * bx + py * by) / b_along - b_along

    def _maybe_detect_virtual_b_crossing(self, gps):
        # Raw and fused GPS callbacks share this detector under a reentrant
        # callback group. Serialize the one-shot transition before spawning the
        # worker so both streams cannot start competing updates.
        with self._dynamic_state_lock:
            self._maybe_detect_virtual_b_crossing_locked(gps)

    def _maybe_detect_virtual_b_crossing_locked(self, gps):
        if (
            not self.dynamic_r_enabled
            or self.mission_type != 'COMPOSITE'
            or self.composite_route_points is None
            or self._b_crossing_triggered
            or self._dynamic_update_cancelled
        ):
            return
        signed_m = self._virtual_b_signed_distance_m(gps)
        if signed_m is None:
            return
        previous = self._b_previous_signed_m
        if previous is None:
            self._b_previous_signed_m = signed_m
            return
        if not (previous < 0.0 <= signed_m):
            self._b_previous_signed_m = signed_m
            return

        current_seq = self._current_mission_seq()
        r_seq = self.dynamic_indices.get('R')
        u_seq = self.dynamic_indices.get('U')
        if current_seq is None or r_seq is None or current_seq >= r_seq:
            self._b_crossing_triggered = True
            self._dynamic_update_result = 'UPDATE_TOO_LATE'
            self._dynamic_update_state = 'TOO_LATE'
            self._dynamic_update_cancelled = True
            self._dynamic_worker_cancel_reason = 'b_trigger_too_late'
            self._publish_aburcd_event(
                'dynamic_update_too_late',
                r_source='SAFE',
                current_seq=current_seq,
                reached_seq=None,
                failure_reason='virtual B crossed after R became current',
                **self._gps_snapshot_fields(gps),
            )
            return
        if current_seq != u_seq:
            # Retain the negative-side sample so a slightly delayed
            # MISSION_CURRENT update can authorize crossing on the next GPS.
            return

        detected = time.monotonic()
        with self._gps_lock:
            prediction_samples = [
                dict(sample)
                for sample in self.gps_history
                if detected - float(sample['timestamp_monotonic'])
                <= self.dynamic_r_prediction_window_sec
            ]
        snapshot = {
            'timestamp_unix_sec': time.time(),
            'timestamp_monotonic': detected,
            'current_seq': current_seq,
            'prediction_samples': prediction_samples,
            **self._gps_snapshot_fields(gps),
        }
        snapshot['snapshot_latency_ms'] = (
            time.monotonic() - detected
        ) * 1000.0
        self.b_frozen_snapshot = dict(snapshot)
        self._b_crossing_triggered = True
        self._dynamic_update_started = True
        self._dynamic_update_state = 'CALCULATING'
        self._dynamic_worker_cancel_reason = None
        self._publish_aburcd_event(
            'b_crossed',
            current_seq=current_seq,
            reached_seq=None,
            b_signed_distance_m=signed_m,
            **self._gps_snapshot_fields(gps),
        )
        self._publish_aburcd_event(
            'b_state_frozen',
            current_seq=current_seq,
            reached_seq=None,
            snapshot_latency_ms=snapshot['snapshot_latency_ms'],
            **self._gps_snapshot_fields(gps),
        )
        worker = threading.Thread(
            target=self._dynamic_r_worker,
            args=(snapshot,),
            name='dynamic-r-update',
            daemon=True,
        )
        worker.start()

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
            self._b_previous_signed_m = (
                self._virtual_b_signed_distance_m(self.get_best_gps())
                if self.get_best_gps() is not None
                else None
            )
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
        self._cancel_dynamic_update('composite_complete')
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
        """Publish a JSON event for summary logging and mission fault handling."""
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
        now_monotonic = time.monotonic()
        now_unix = time.time()
        with self._live_state_lock:
            live_vfr = self.live_vehicle_state.get('vfr_hud')
            vfr_timestamp = self.live_vehicle_state.get('vfr_timestamp_monotonic')
            rel_alt_m = self.live_vehicle_state.get('rel_alt_m')
            rel_alt_timestamp = self.live_vehicle_state.get(
                'rel_alt_timestamp_monotonic'
            )
            live_vfr = dict(live_vfr) if live_vfr is not None else None
        sample = {
            'timestamp_unix_sec': now_unix,
            'timestamp_monotonic': now_monotonic,
            'lat': float(msg.latitude),
            'lon': float(msg.longitude),
            'ground_speed_mps': None,
            'vertical_speed_mps': None,
            'heading_deg': None,
            'relative_altitude_m': None,
        }
        if (
            live_vfr is not None
            and vfr_timestamp is not None
            and now_monotonic - float(vfr_timestamp) <= self.gps_stale_timeout_sec
        ):
            sample.update(live_vfr)
        if (
            rel_alt_m is not None
            and rel_alt_timestamp is not None
            and now_monotonic - float(rel_alt_timestamp) <= self.gps_stale_timeout_sec
            and math.isfinite(float(rel_alt_m))
        ):
            sample['relative_altitude_m'] = float(rel_alt_m)
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
        for sample in gps_samples:
            sample_time, latitude, longitude = self._gps_history_basic(sample)
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
    def _gps_history_basic(sample):
        """Return (unix_time, lat, lon) for dict or legacy tuple samples."""
        if isinstance(sample, dict):
            return (
                float(sample['timestamp_unix_sec']),
                float(sample['lat']),
                float(sample['lon']),
            )
        return float(sample[0]), float(sample[1]), float(sample[2])

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
        latest_age_sec = now - self._gps_history_basic(gps_samples[-1])[0]
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
        for sample in gps_samples:
            sample_time, latitude, longitude = self._gps_history_basic(sample)
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
        """Compute ABURCD geometry while B and C remain virtual points."""
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

        u_lat, u_lon = self.destination_point(
            target_lat,
            target_lon,
            reverse_heading_deg,
            self.u_offset_m,
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
            'U': {'lat': u_lat, 'lon': u_lon},
            'R': {'lat': r_lat, 'lon': r_lon},
            'C': {'lat': c_lat, 'lon': c_lon},
            'D': {'lat': d_lat, 'lon': d_lon},
            'heading_deg': heading_deg,
            'reverse_heading_deg': reverse_heading_deg
        }

    @staticmethod
    def _weighted_mean(values, weights):
        total_weight = sum(weights)
        if total_weight <= 0.0:
            raise ValueError('non_positive_weight_sum')
        return sum(value * weight for value, weight in zip(values, weights)) / total_weight

    @staticmethod
    def _weighted_linear_fit_at_zero(times, values, weights):
        """Return intercept at t=0, slope and weighted RMSE."""
        if not (len(times) == len(values) == len(weights)) or len(times) < 2:
            raise ValueError('insufficient_fit_samples')
        total_weight = sum(weights)
        if total_weight <= 0.0:
            raise ValueError('non_positive_weight_sum')
        mean_t = sum(weight * value for weight, value in zip(weights, times)) / total_weight
        mean_y = sum(weight * value for weight, value in zip(weights, values)) / total_weight
        denominator = sum(
            weight * (value - mean_t) ** 2
            for weight, value in zip(weights, times)
        )
        if denominator <= 1.0e-9:
            raise ValueError('fit_time_span_too_small')
        slope = sum(
            weight * (time_value - mean_t) * (measurement - mean_y)
            for weight, time_value, measurement in zip(weights, times, values)
        ) / denominator
        intercept = mean_y - slope * mean_t
        rmse = math.sqrt(
            sum(
                weight * (
                    measurement - (intercept + slope * time_value)
                ) ** 2
                for weight, time_value, measurement
                in zip(weights, times, values)
            ) / total_weight
        )
        return intercept, slope, rmse

    @staticmethod
    def _signed_heading_error_deg(actual_heading_deg, planned_heading_deg):
        return (
            (float(actual_heading_deg) - float(planned_heading_deg) + 180.0)
            % 360.0
            - 180.0
        )

    def _dynamic_prediction_inputs(self, b_snapshot):
        samples = list((b_snapshot or {}).get('prediction_samples') or [])
        if len(samples) < self.dynamic_r_min_prediction_samples:
            return None, 'prediction_samples_insufficient'
        samples.sort(key=lambda sample: float(sample['timestamp_monotonic']))
        b_time = float(b_snapshot['timestamp_monotonic'])
        valid = []
        for sample in samples:
            timestamp = sample.get('timestamp_monotonic')
            ground_speed = sample.get('ground_speed_mps')
            vertical_speed = sample.get('vertical_speed_mps')
            heading = sample.get('heading_deg')
            lat = sample.get('lat')
            lon = sample.get('lon')
            values = (timestamp, ground_speed, vertical_speed, heading, lat, lon)
            if any(value is None for value in values):
                continue
            if not all(math.isfinite(float(value)) for value in values):
                continue
            age = b_time - float(timestamp)
            if age < -0.05 or age > self.dynamic_r_prediction_window_sec + 0.05:
                continue
            valid.append(sample)
        if len(valid) < self.dynamic_r_min_prediction_samples:
            return None, 'prediction_motion_samples_insufficient'
        span_sec = (
            float(valid[-1]['timestamp_monotonic'])
            - float(valid[0]['timestamp_monotonic'])
        )
        if span_sec < self.dynamic_r_min_prediction_span_sec:
            return None, 'prediction_sample_span_too_short'

        relative_altitude_m = b_snapshot.get('relative_altitude_m')
        relative_altitude_age_sec = b_snapshot.get('relative_altitude_age_sec')
        if (
            relative_altitude_m is None
            or not math.isfinite(float(relative_altitude_m))
            or float(relative_altitude_m) <= 0.0
        ):
            return None, 'relative_altitude_missing_or_invalid'
        if (
            relative_altitude_age_sec is None
            or not math.isfinite(float(relative_altitude_age_sec))
            or float(relative_altitude_age_sec) > self.gps_stale_timeout_sec
        ):
            return None, 'relative_altitude_stale'

        attack_heading_deg = float(self.composite_route_points['heading_deg'])
        a_point = self.composite_route_points['A']
        c_point = self.composite_route_points['C']
        line_x, line_y = self.local_xy_m(
            a_point['lat'], a_point['lon'], c_point['lat'], c_point['lon']
        )
        line_length = math.hypot(line_x, line_y)
        if line_length < 1.0:
            return None, 'attack_line_too_short'
        unit_x = line_x / line_length
        unit_y = line_y / line_length

        weights = [float(index + 1) for index in range(len(valid))]
        v_forward_values = []
        vertical_speed_values = []
        heading_errors = []
        cross_tracks = []
        fit_times = []
        for sample in valid:
            heading_error_deg = self._signed_heading_error_deg(
                sample['heading_deg'], attack_heading_deg
            )
            v_forward = float(sample['ground_speed_mps']) * math.cos(
                math.radians(heading_error_deg)
            )
            x_m, y_m = self.local_xy_m(
                a_point['lat'], a_point['lon'],
                float(sample['lat']), float(sample['lon']),
            )
            cross_track_m = abs(x_m * unit_y - y_m * unit_x)
            v_forward_values.append(v_forward)
            vertical_speed_values.append(float(sample['vertical_speed_mps']))
            heading_errors.append(heading_error_deg)
            cross_tracks.append(cross_track_m)
            fit_times.append(float(sample['timestamp_monotonic']) - b_time)

        v_forward_est = self._weighted_mean(v_forward_values, weights)
        if not math.isfinite(v_forward_est) or v_forward_est <= 1.0:
            return None, 'forward_speed_invalid'

        vertical_speed_mean = self._weighted_mean(vertical_speed_values, weights)
        try:
            vz_at_b_fit, az_fit, vz_fit_rmse = self._weighted_linear_fit_at_zero(
                fit_times, vertical_speed_values, weights
            )
        except ValueError:
            vz_at_b_fit = vertical_speed_mean
            az_fit = 0.0
            vz_fit_rmse = float('inf')

        trend_valid = (
            math.isfinite(vz_at_b_fit)
            and math.isfinite(az_fit)
            and math.isfinite(vz_fit_rmse)
            and vz_fit_rmse <= self.dynamic_r_vz_fit_max_rmse_mps
            and abs(az_fit) <= self.dynamic_r_max_abs_vertical_accel_mps2
        )
        if trend_valid:
            vz_estimation_mode = 'trend'
            vz_at_b_est = vz_at_b_fit
            az_est = az_fit
        else:
            vz_estimation_mode = 'weighted_mean'
            vz_at_b_est = vertical_speed_mean
            az_est = 0.0

        return {
            'sample_count': len(valid),
            'window_sec': span_sec,
            'height_source': 'rel_alt',
            'height_b_m': float(relative_altitude_m),
            'v_forward_mean_mps': v_forward_est,
            'vz_est_mps': vz_at_b_est,
            'vz_trend_mps2': az_est,
            'vz_fit_rmse_mps': vz_fit_rmse,
            'vz_estimation_mode': vz_estimation_mode,
            'heading_error_mean_deg': self._weighted_mean(heading_errors, weights),
            'cross_track_mean_m': self._weighted_mean(cross_tracks, weights),
        }, 'ok'

    def _compute_predicted_dynamic_r(self, b_snapshot, target_c):
        """Compute dynamic R from AB samples and the configured attack altitude.

        AB samples are used to estimate the forward ground speed (therefore
        naturally capturing first-order headwind/tailwind effects).  The
        ballistic release state is constrained by the commanded attack
        altitude instead of extrapolating the short AB vertical-speed trend
        all the way to R.  ArduPlane is actively tracking the mission altitude
        on U/R, so the release-state model assumes level flight at R:

            h_R = mission_altitude_m
            vz_R = 0

        The measured/fitted AB vertical state remains in ``prediction`` for
        diagnostics and later model refinement, but it does not bias the
        ballistic height calculation.
        """
        prediction, reason = self._dynamic_prediction_inputs(b_snapshot)
        if prediction is None:
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': reason,
            }

        rc_min = self.dynamic_r_min_rc_m
        rc_max = self.u_offset_m - self.dynamic_r_min_u_r_distance_m
        if rc_max <= rc_min:
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': 'dynamic_rc_bounds_invalid',
                'prediction': prediction,
            }

        predicted_height_r_m = float(self.mission_altitude_m)
        predicted_vz_r_mps = 0.0
        if not math.isfinite(predicted_height_r_m) or predicted_height_r_m <= 0.0:
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': 'mission_altitude_invalid_for_ballistics',
                'prediction': prediction,
            }

        g_mps2 = 9.80665
        fall_time_sec = math.sqrt(2.0 * predicted_height_r_m / g_mps2)
        if not math.isfinite(fall_time_sec) or fall_time_sec <= 0.0:
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': 'fall_time_invalid',
                'prediction': prediction,
            }

        rc_new = prediction['v_forward_mean_mps'] * (
            fall_time_sec + self.dynamic_r_release_delay_sec
        )
        if not math.isfinite(rc_new):
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': 'rc_non_finite',
                'prediction': prediction,
            }
        if not rc_min <= rc_new <= rc_max:
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': 'rc_out_of_range',
                'prediction': prediction,
                'rc_dynamic_m': rc_new,
            }

        # B->R time is retained as a diagnostic only.  It no longer feeds a
        # free vertical extrapolation because R is altitude-controlled.
        distance_b_to_r_m = self.b_offset_m - rc_new
        if distance_b_to_r_m <= 0.0:
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': 'predicted_r_not_after_b',
                'prediction': prediction,
                'rc_dynamic_m': rc_new,
            }
        t_br_sec = distance_b_to_r_m / prediction['v_forward_mean_mps']

        final_state = {
            'predicted_height_r_m': predicted_height_r_m,
            'predicted_vz_r_mps': predicted_vz_r_mps,
            'predicted_v_forward_r_mps': prediction['v_forward_mean_mps'],
            'fall_time_sec': fall_time_sec,
            'vertical_prediction_mode': 'mission_altitude_level',
            'vertical_zero_crossing_sec': None,
            'commanded_altitude_r_m': predicted_height_r_m,
        }
        iterations = [{
            'iteration': 1,
            'rc_guess_m': self.release_offset_m,
            'distance_b_to_r_m': distance_b_to_r_m,
            't_br_sec': t_br_sec,
            **final_state,
            'rc_new_m': rc_new,
            'delta_rc_m': abs(rc_new - float(self.release_offset_m)),
        }]

        lat, lon = self.destination_point(
            float(target_c['lat']),
            float(target_c['lon']),
            self.composite_route_points['reverse_heading_deg'],
            rc_new,
        )
        prediction.update(final_state)
        prediction.update({
            'iteration_count': 1,
            'converged': True,
            'rc_dynamic_m': rc_new,
            'release_delay_sec': self.dynamic_r_release_delay_sec,
        })
        return {
            'valid': True,
            'lat': lat,
            'lon': lon,
            'alt': self.mission_altitude_m,
            'reason': 'ab_multisample_prediction',
            'prediction': prediction,
            'iterations': iterations,
            'rc_dynamic_m': rc_new,
        }

    def compute_dynamic_r(self, b_snapshot, target_c):
        """Return a dynamic-R candidate, separate from callback policy."""
        if not self.dynamic_r_enabled:
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': 'dynamic_r_disabled',
            }
        if not b_snapshot or target_c is None:
            return {
                'valid': False,
                'lat': None,
                'lon': None,
                'alt': None,
                'reason': 'dynamic_r_input_missing',
            }
        if self.dynamic_r_test_mode:
            lat, lon = self.destination_point(
                float(target_c['lat']),
                float(target_c['lon']),
                self.composite_route_points['reverse_heading_deg'],
                self.dynamic_r_test_offset_m,
            )
            return {
                'valid': True,
                'lat': lat,
                'lon': lon,
                'alt': self.mission_altitude_m,
                'reason': 'TEST_ONLY_fixed_offset',
                'rc_dynamic_m': self.dynamic_r_test_offset_m,
            }
        return self._compute_predicted_dynamic_r(b_snapshot, target_c)

    def validate_dynamic_r_candidate(self, candidate):
        if not candidate.get('valid'):
            return False, str(candidate.get('reason', 'dynamic_r_invalid'))
        values = [
            candidate.get('lat'), candidate.get('lon'), candidate.get('alt')
        ]
        if not all(
            value is not None and math.isfinite(float(value))
            for value in values
        ):
            return False, 'dynamic_r_non_finite'
        points = self.composite_route_points
        ux, uy = self.local_xy_m(
            points['U']['lat'], points['U']['lon'],
            points['C']['lat'], points['C']['lon'],
        )
        rx, ry = self.local_xy_m(
            points['U']['lat'], points['U']['lon'],
            float(candidate['lat']), float(candidate['lon']),
        )
        length = math.hypot(ux, uy)
        if length < 1.0:
            return False, 'u_c_segment_too_short'
        along = (rx * ux + ry * uy) / length
        cross = abs(rx * uy - ry * ux) / length
        if not (0.0 < along < length) or cross > 1.0:
            return False, (
                'R_DYNAMIC_OUT_OF_RANGE:'
                f'along_u_to_c_m={along:.3f}:u_c_length_m={length:.3f}:'
                f'cross_track_m={cross:.3f}'
            )
        rc_distance_m = self.distance_m(
            float(candidate['lat']), float(candidate['lon']),
            points['C']['lat'], points['C']['lon'],
        )
        rc_max_m = self.u_offset_m - self.dynamic_r_min_u_r_distance_m
        if not self.dynamic_r_min_rc_m <= rc_distance_m <= rc_max_m:
            return False, (
                'R_DYNAMIC_OUT_OF_RANGE:'
                f'rc_distance_m={rc_distance_m:.3f}:'
                f'rc_min_m={self.dynamic_r_min_rc_m:.3f}:'
                f'rc_max_m={rc_max_m:.3f}'
            )
        return True, 'dynamic_r_between_u_and_c'

    def _dynamic_update_timed_out(self, started_monotonic):
        return (
            time.monotonic() - started_monotonic
            > self.dynamic_r_update_timeout_sec
        )

    def _dynamic_r_worker(self, snapshot):
        if self._second_full_push_attempted:
            return
        started = float(snapshot['timestamp_monotonic'])
        self._dynamic_update_state = 'CALCULATING'
        self._publish_aburcd_event(
            'r_calc_start',
            current_seq=self._current_mission_seq(),
            reached_seq=None,
            **self._gps_snapshot_fields(),
        )
        calc_started = time.monotonic()
        self._dynamic_metrics.update(dynamic_trigger_timestamp=started,
                                     r_calc_start=calc_started)
        try:
            candidate = self.compute_dynamic_r(
                snapshot,
                self.composite_route_points['C'],
            )
        except Exception as error:  # noqa: BLE001 - future algorithm boundary.
            candidate = {'valid': False, 'reason': f'exception:{error}'}
        self._dynamic_metrics['r_calc_done'] = time.monotonic()
        calc_duration_ms = (time.monotonic() - calc_started) * 1000.0
        prediction = candidate.get('prediction') or {}
        if prediction:
            self._publish_aburcd_event(
                'ab_prediction_input',
                current_seq=self._current_mission_seq(),
                reached_seq=None,
                sample_count=prediction.get('sample_count'),
                window_sec=prediction.get('window_sec'),
                height_source=prediction.get('height_source'),
                height_b_m=prediction.get('height_b_m'),
                v_forward_mean_mps=prediction.get('v_forward_mean_mps'),
                vz_est_mps=prediction.get('vz_est_mps'),
                vz_trend_mps2=prediction.get('vz_trend_mps2'),
                vz_fit_rmse_mps=(
                    prediction.get('vz_fit_rmse_mps')
                    if math.isfinite(float(prediction.get('vz_fit_rmse_mps', float('nan'))))
                    else None
                ),
                vz_estimation_mode=prediction.get('vz_estimation_mode'),
                heading_error_mean_deg=prediction.get('heading_error_mean_deg'),
                cross_track_mean_m=prediction.get('cross_track_mean_m'),
            )
        for iteration in candidate.get('iterations') or []:
            self._publish_aburcd_event(
                'r_prediction_iteration',
                current_seq=self._current_mission_seq(),
                reached_seq=None,
                **iteration,
            )
        valid, reason = self.validate_dynamic_r_candidate(candidate)
        if not valid:
            if not self._set_dynamic_update_result('R_SAFE_FALLBACK'):
                return
            failure_event = (
                'r_dynamic_out_of_range'
                if str(reason).startswith('R_DYNAMIC_OUT_OF_RANGE')
                else 'r_calc_failed'
            )
            self._publish_aburcd_event(
                failure_event,
                current_seq=self._current_mission_seq(),
                reached_seq=None,
                calc_duration_ms=calc_duration_ms,
                failure_reason=reason,
                fallback='R_safe retained',
                **self._gps_snapshot_fields(),
            )
            self._full_update_event('dynamic_update_skipped',
                                    reason='r_calculation_failed', r_source='SAFE')
            return
        try:
            self._require_dynamic_operation_allowed(started, 'after_calculation')
        except DynamicUpdateAborted as error:
            self._finish_dynamic_abort(str(error), started)
            return
        self._publish_aburcd_event(
            'r_calc_done',
            current_seq=self._current_mission_seq(),
            reached_seq=None,
            calc_duration_ms=calc_duration_ms,
            dynamic_r_lat=float(candidate['lat']),
            dynamic_r_lon=float(candidate['lon']),
            dynamic_r_alt=float(candidate['alt']),
            calculation_reason=candidate['reason'],
            prediction_mode=prediction.get('vz_estimation_mode'),
            iteration_count=prediction.get('iteration_count'),
            converged=prediction.get('converged'),
            rc_dynamic_m=candidate.get('rc_dynamic_m'),
            predicted_height_r_m=prediction.get('predicted_height_r_m'),
            predicted_vz_r_mps=prediction.get('predicted_vz_r_mps'),
            predicted_v_forward_mps=prediction.get('predicted_v_forward_r_mps'),
            vertical_prediction_mode=prediction.get('vertical_prediction_mode'),
            vertical_zero_crossing_sec=prediction.get('vertical_zero_crossing_sec'),
            fall_time_sec=prediction.get('fall_time_sec'),
            release_delay_sec=(
                prediction.get('release_delay_sec')
                if prediction else self.dynamic_r_release_delay_sec
            ),
            **self._gps_snapshot_fields(),
        )
        if (
            candidate.get('reason') == 'ab_multisample_prediction'
            and self.dynamic_r_prediction_shadow_mode
        ):
            self._dynamic_prediction_shadow = copy.deepcopy(prediction)
            if self._set_dynamic_update_result('R_SAFE_FALLBACK'):
                self._publish_aburcd_event(
                    'r_calc_shadow',
                    current_seq=self._current_mission_seq(),
                    reached_seq=None,
                    rc_dynamic_m=candidate.get('rc_dynamic_m'),
                    fallback='R_safe retained',
                    reason='prediction_shadow_mode',
                )
                self._full_update_event(
                    'dynamic_update_skipped',
                    reason='prediction_shadow_mode',
                    r_source='SAFE',
                )
            return
        if not self._mission_update_lock.acquire(blocking=False):
            if not self._set_dynamic_update_result('UPDATE_FAILED'):
                return
            self._publish_aburcd_event(
                'r_update_rejected_mission_busy',
                failure_reason='mission update transaction already active',
                **self._gps_snapshot_fields(),
            )
            return
        try:
            asyncio.run(
                self._update_dynamic_r_async(
                    candidate, snapshot, calc_duration_ms
                )
            )
        except DynamicUpdateAborted as error:
            self._finish_dynamic_abort(str(error), started)
        except Exception as error:
            self._dynamic_manual_failure('UNKNOWN', str(error))
        finally:
            self._mission_update_lock.release()

    @property
    def safe_composite_mission(self):
        """Return a defensive copy of the first verified upload."""
        return copy.deepcopy(self._safe_composite_mission)

    def build_dynamic_mission(self, candidate):
        safe = self.safe_composite_mission
        if safe is None:
            raise RuntimeError('verified safe composite mission unavailable')
        expected = list(copy.deepcopy(safe))
        expected[int(self.dynamic_indices['R'])] = self.make_nav_waypoint(
            candidate['lat'], candidate['lon'], candidate['alt'],
            self.c_acceptance_radius_m,
        )
        return expected

    def dynamic_mission_structure_matches(self, expected):
        safe = self.safe_composite_mission
        if safe is None or len(safe) != len(expected):
            return False
        r_seq = int(self.dynamic_indices['R'])
        if not 0 <= r_seq < len(safe):
            return False
        fields = ('frame', 'command', 'param1', 'param2', 'param3', 'param4',
                  'x_lat', 'y_long', 'z_alt', 'autocontinue', 'is_current')
        return all(
            all(getattr(old, field) == getattr(new, field) for field in fields)
            for seq, (old, new) in enumerate(zip(safe, expected)) if seq != r_seq
        )

    def _full_update_event(self, event, **fields):
        record = dict(self._dynamic_metrics)
        record.update(fields)
        self._publish_aburcd_event(event, **record)

    def _dynamic_manual_failure(self, state, reason, event=None):
        self._cancel_dynamic_update(reason)
        self._set_dynamic_update_result('UPDATE_FAILED')
        self._full_update_event(
            event or ('mission_inconsistent' if state == 'INCONSISTENT'
                      else 'mission_state_unknown'),
            mission_state=state, r_source='UNKNOWN', failure_reason=reason,
            requires_manual_intervention=True,
        )
        # The manager routes this event to its existing fail()/enter_safe().
        self.publish_mission_summary_event(
            'dynamic_mission_failed', mission_state=state, reason=reason,
            requires_manual_intervention=True,
        )

    async def _check_dynamic_progress(self, started, safe_content=False):
        metrics = self._dynamic_metrics
        with self._dynamic_state_lock:
            before = metrics['current_seq_before_update']
            current = self._current_mission_seq()
            reached = max(self.last_reached_seq,
                          metrics['last_reached_seq_before_update'])
            r_seq = self.dynamic_indices['R']
            metrics.update(current_seq_after_update=current,
                           last_reached_seq_after_update=reached)
            self._full_update_event('mission_progress_check',
                                    dynamic_update_state=self._dynamic_update_state)
            if not safe_content:
                reason = self._dynamic_abort_reason_locked(started)
                if reason:
                    raise DynamicUpdateAborted(f'progress_check:{reason}')
            if (before is None or current is None or before < 1 or current < 0
                    or current >= len(self._safe_composite_mission)):
                return False
            # Reached(U) can arrive before the next MISSION_CURRENT. Equality
            # is normal; only an actual decrease from before requires recovery.
            r_still_future = current < r_seq and reached < r_seq
            if current >= before and (
                    r_still_future or (safe_content and current > reached)):
                self._full_update_event(
                    'mission_progress_safe',
                    reason=('r_still_future_natural_progress' if r_still_future
                            else 'safe_mission_natural_progress'))
                self._full_update_event('mission_progress_accepted', action='no_set_current')
                return True
            if current >= before:
                return False
            reason = self._dynamic_abort_reason_locked(started)
            if reason:
                raise DynamicUpdateAborted(f'progress_recovery:{reason}')
            # A genuine regression is recoverable only with reached evidence.
            resume = max(before, reached + 1, current)
            if reached < 0 or not (reached < resume < r_seq):
                return False
            self._full_update_event('mission_progress_recovery', resume_seq=resume)
        await self.set_current_mission_item_async(resume, dynamic_started=started)
        deadline = time.monotonic() + self.service_timeout_sec
        while time.monotonic() < deadline:
            with self._dynamic_state_lock:
                reason = self._dynamic_abort_reason_locked(started)
                if reason:
                    raise DynamicUpdateAborted(f'recovery_confirm:{reason}')
                current = self._current_mission_seq()
                reached = self.last_reached_seq
                if current is not None and resume <= current < r_seq and reached < r_seq:
                    metrics.update(current_seq_after_update=current,
                                   last_reached_seq_after_update=reached)
                    return True
            await asyncio.sleep(0.05)
        return False

    async def _update_dynamic_r_async(self, candidate, snapshot, calc_duration_ms):
        """Replace the complete FCU mission once with the dynamic-R version.

        The first upload installs the complete R_safe mission.  This second
        transaction again uses WaypointPush(start_index=0); no partial mission
        protocol or raw MAVLink writer is used.
        """
        started = float(snapshot['timestamp_monotonic'])
        self._require_dynamic_operation_allowed(started, 'before_second_full_push')
        if self._second_full_push_attempted:
            return

        expected = self.build_dynamic_mission(candidate)
        if not self.dynamic_mission_structure_matches(expected):
            self._full_update_event(
                'dynamic_mission_structure_mismatch', r_source='SAFE'
            )
            self._cancel_dynamic_update('structure_mismatch')
            return

        if self._current_mission_seq() != self.dynamic_indices['U']:
            self._dynamic_manual_failure(
                'SAFE',
                'current navigation item is not U',
                'second_full_push_failed',
            )
            return

        with self._dynamic_state_lock:
            self._dynamic_metrics.update(
                dynamic_trigger_timestamp=started,
                calc_duration_ms=calc_duration_ms,
                current_seq_before_update=self._current_mission_seq(),
                last_reached_seq_before_update=self.last_reached_seq,
                second_full_push_start=time.monotonic(),
            )

        self._require_dynamic_operation_allowed(started, 'second_full_push_dispatch')
        self._second_full_push_attempted = True
        self._dynamic_update_state = 'PUSHING'
        push_started = time.monotonic()
        self._full_update_event(
            'second_full_push_start',
            waypoint_count=len(expected),
            start_index=0,
            r_seq=int(self.dynamic_indices['R']),
        )
        try:
            # Deliberately no service retry for the in-flight replacement.
            await self.push_mission_async(
                expected, retry=False, started_monotonic=started
            )
        except Exception as error:
            self._dynamic_metrics['second_full_push_failure_reason'] = str(error)
            await self._handle_dynamic_full_update_failure_async(
                candidate,
                expected,
                started,
                f'second_full_push_failed:{error}',
                calc_duration_ms=calc_duration_ms,
            )
            return

        push_duration_ms = (time.monotonic() - push_started) * 1000.0
        self._dynamic_update_state = 'PUSH_ACKED'
        self._dynamic_metrics.update(
            second_full_push_done=time.monotonic(),
            second_full_push_duration_ms=push_duration_ms,
        )
        self._full_update_event(
            'second_full_push_done',
            waypoint_count=len(expected),
            push_duration_ms=push_duration_ms,
            second_full_push_duration_ms=push_duration_ms,
        )

        self._require_dynamic_operation_allowed(started, 'before_second_full_pull')
        self._dynamic_update_state = 'VERIFYING'
        pull_started = time.monotonic()
        self._dynamic_metrics['second_full_pull_start'] = pull_started
        self._full_update_event('second_full_pull_start')
        try:
            pulled = await self._pull_dynamic_mission_async(started)
        except Exception as error:
            await self._handle_dynamic_full_update_failure_async(
                candidate,
                expected,
                started,
                f'second_full_pull_failed:{error}',
                calc_duration_ms=calc_duration_ms,
                push_duration_ms=push_duration_ms,
            )
            return

        pull_duration_ms = (time.monotonic() - pull_started) * 1000.0
        self._dynamic_metrics.update(
            second_full_pull_done=time.monotonic(),
            second_full_pull_duration_ms=pull_duration_ms,
        )
        self._full_update_event(
            'second_full_pull_done',
            second_full_pull_duration_ms=pull_duration_ms,
        )

        self._require_dynamic_operation_allowed(started, 'before_second_full_verify')
        verify_started = time.monotonic()
        self._dynamic_metrics['second_full_verify_start'] = verify_started
        self._full_update_event('second_full_verify_start')
        actual = list(pulled.waypoints)
        ok, reason = self.verify_temporary_mission(expected, actual)
        verify_cpu_duration_ms = (time.monotonic() - verify_started) * 1000.0
        self._dynamic_metrics.update(
            second_full_verify_done=time.monotonic(),
            verify_cpu_duration_ms=verify_cpu_duration_ms,
        )
        if not ok:
            await self._handle_dynamic_full_update_failure_async(
                candidate,
                expected,
                started,
                f'second_full_verify_failed:{reason}',
                calc_duration_ms=calc_duration_ms,
                push_duration_ms=push_duration_ms,
                verify_duration_ms=pull_duration_ms,
                verify_cpu_duration_ms=verify_cpu_duration_ms,
            )
            return

        self._full_update_event(
            'second_full_verify_pass', verification_reason=reason
        )

        progress_ok = await self._check_dynamic_progress(started)
        if not progress_ok:
            self._dynamic_manual_failure(
                'UNKNOWN',
                'mission progress could not be verified after second full upload',
                'mission_progress_unknown',
            )
            return

        self._require_dynamic_operation_allowed(started, 'final_verify_commit')
        committed = self._commit_dynamic_r_verified(
            candidate, expected, started, calc_duration_ms,
            push_duration_ms, pull_duration_ms, reason,
            verify_cpu_duration_ms,
        )
        if not committed:
            self._finish_dynamic_abort('deadline changed before commit', started)

    async def _pull_dynamic_mission_async(self, started, *, reconcile=False):
        """Pull the complete FCU mission.

        Normal verification obeys the dynamic deadline.  Reconciliation after a
        failed second full push is read-only and is therefore allowed to finish
        even if the write deadline has just expired.
        """
        if not reconcile:
            self._require_dynamic_operation_allowed(started, 'pull')
            remaining = self.dynamic_r_update_timeout_sec - (
                time.monotonic() - started
            )
            return await self.pull_mission_async(
                timeout_sec=max(0.05, remaining),
                abort_check=lambda: self._dynamic_abort_reason(started),
            )
        return await self.pull_mission_async(timeout_sec=self.service_timeout_sec)

    def _classify_actual_mission(self, pulled, dynamic_expected):
        if pulled is None or getattr(pulled, 'waypoints', None) is None:
            return 'UNKNOWN', 'mission unavailable'
        actual = list(pulled.waypoints)
        safe_ok, safe_reason = self.verify_temporary_mission(
            list(self.safe_composite_mission), actual
        )
        if safe_ok:
            return 'SAFE', safe_reason
        dynamic_ok, dynamic_reason = self.verify_temporary_mission(
            dynamic_expected, actual
        )
        if dynamic_ok:
            return 'DYNAMIC', dynamic_reason
        return (
            'INCONSISTENT',
            f'safe_mismatch={safe_reason};dynamic_mismatch={dynamic_reason}',
        )

    async def _handle_dynamic_full_update_failure_async(
        self, candidate, expected, started, failure_reason, **durations
    ):
        """Reconcile a failed full replacement without issuing another write."""
        mission_state = 'UNKNOWN'
        reconcile_reason = 'pull not attempted'
        pulled = None
        try:
            pulled = await self._pull_dynamic_mission_async(
                started, reconcile=True
            )
            mission_state, reconcile_reason = self._classify_actual_mission(
                pulled, expected
            )
        except Exception as error:
            reconcile_reason = str(error)

        self._dynamic_metrics['second_full_push_failure_reason'] = failure_reason
        self._full_update_event(
            'second_full_push_failed',
            failure_reason=failure_reason,
            mission_state=mission_state,
            reconciliation_reason=reconcile_reason,
            **durations,
        )

        if mission_state == 'SAFE':
            try:
                progress_ok = await self._check_dynamic_progress(
                    started, safe_content=True
                )
            except Exception as error:
                progress_ok = False
                reconcile_reason = f'{reconcile_reason};progress={error}'
            if progress_ok:
                self._set_dynamic_update_result('R_SAFE_FALLBACK')
                self._dynamic_update_state = 'SAFE_RECONCILED'
                self._full_update_event(
                    'dynamic_update_skipped',
                    reason='second_full_push_failed_safe_mission_confirmed',
                    mission_state='SAFE',
                    r_source='SAFE',
                )
                return
            self._dynamic_manual_failure(
                'UNKNOWN',
                f'safe mission confirmed but progress unknown: {reconcile_reason}',
                'mission_progress_unknown',
            )
            return

        if mission_state == 'DYNAMIC':
            # A service error can occur after the FCU accepted the full mission.
            # Treat it as success only if the original deadline still holds and
            # mission progress can be verified; never issue a compensating write.
            try:
                self._require_dynamic_operation_allowed(
                    started, 'reconciled_dynamic_commit'
                )
                progress_ok = await self._check_dynamic_progress(started)
            except Exception as error:
                progress_ok = False
                reconcile_reason = f'{reconcile_reason};progress={error}'
            if progress_ok:
                verify_reason = f'reconciled_after_error:{reconcile_reason}'
                committed = self._commit_dynamic_r_verified(
                    candidate,
                    expected,
                    started,
                    durations.get('calc_duration_ms', 0.0),
                    durations.get('push_duration_ms', 0.0),
                    durations.get('verify_duration_ms', 0.0),
                    verify_reason,
                    durations.get('verify_cpu_duration_ms', 0.0),
                )
                if committed:
                    return
            self._dynamic_manual_failure(
                'DYNAMIC',
                f'dynamic mission present but could not be safely committed: {reconcile_reason}',
                'dynamic_mission_failed',
            )
            return

        self._dynamic_manual_failure(
            mission_state,
            f'{failure_reason}; reconciliation={reconcile_reason}',
            'mission_inconsistent' if mission_state == 'INCONSISTENT'
            else 'mission_state_unknown',
        )

    def _commit_dynamic_r_verified(
        self,
        candidate,
        expected,
        started,
        calc_duration_ms,
        push_duration_ms,
        verify_duration_ms,
        verification_reason,
        verify_cpu_duration_ms=0.0,
    ):
        """Atomically choose the only successful dynamic-R terminal path."""
        with self._dynamic_state_lock:
            # Fresh live-cache read: never use pulled.current_seq for this
            # final success decision.
            deadline_reason, live_current_seq, reached_seq = (
                self._dynamic_deadline_evidence_locked()
            )
            if (
                self._dynamic_update_result == 'UPDATE_TOO_LATE'
                or self._r_active_before_verify_reported
                or deadline_reason is not None
            ):
                if deadline_reason in {
                    'r_reached', 'mission_current_r'
                }:
                    self._dynamic_update_result = 'UPDATE_TOO_LATE'
                    self._dynamic_update_state = 'TOO_LATE'
                    self._dynamic_update_cancelled = True
                    self._dynamic_worker_cancel_reason = deadline_reason
                    self._full_update_event('dynamic_update_too_late',
                                            failure_reason=deadline_reason)
                return False
            if self._dynamic_update_timed_out(started):
                self._dynamic_update_state = 'FAILED'
                self._dynamic_update_result = 'UPDATE_FAILED'
                self._dynamic_update_cancelled = True
                self._dynamic_worker_cancel_reason = 'dynamic_update_timeout'
                return False
            # Reached may equal current while R is still future. Reject only
            # missing/invalid telemetry or a new regression since progress check.
            accepted_seq = self._dynamic_metrics.get('current_seq_after_update',
                                                      live_current_seq)
            if (live_current_seq is None or live_current_seq < 1
                    or (accepted_seq is not None and live_current_seq < accepted_seq)):
                self._dynamic_update_result = 'UPDATE_FAILED'
                self._dynamic_update_state = 'FAILED'
                self._publish_aburcd_event(
                    'mission_progress_unknown',
                    current_seq=live_current_seq,
                    reached_seq=reached_seq,
                    failure_reason=(
                        'final live current-seq is missing or regressed'
                    ),
                    verify_duration_ms=verify_duration_ms,
                    **self._gps_snapshot_fields(),
                )
                return False

            verified_at = time.monotonic()
            self.composite_expected_waypoints = expected
            self.composite_route_points['R'] = {
                'lat': float(candidate['lat']),
                'lon': float(candidate['lon']),
            }
            self._dynamic_prediction_commit = copy.deepcopy(
                candidate.get('prediction')
            )
            gps = self.get_best_gps()
            self._dynamic_update_verified = True
            self._dynamic_update_verified_monotonic = verified_at
            self._dynamic_update_verified_gps = (
                {'lat': float(gps.latitude), 'lon': float(gps.longitude)}
                if gps is not None else None
            )
            self._dynamic_update_result = 'DYNAMIC_R'
            self._dynamic_update_state = 'VERIFIED'
            total_ms = (verified_at - started) * 1000.0
            self._dynamic_metrics.update(
                current_seq_after_update=live_current_seq,
                last_reached_seq_after_update=reached_seq,
                dynamic_full_update_total_ms=total_ms,
            )
            # Publish before releasing the state lock, so MISSION_CURRENT_R
            # cannot be emitted first by waypoints_callback().
            self._full_update_event(
                'dynamic_mission_verified',
                r_source='DYNAMIC', mission_state='DYNAMIC',
                dynamic_full_update_total_ms=total_ms,
                current_seq=live_current_seq,
                reached_seq=None,
                calc_duration_ms=calc_duration_ms,
                push_duration_ms=push_duration_ms,
                verify_duration_ms=verify_duration_ms,
                verify_cpu_duration_ms=verify_cpu_duration_ms,
                deadline_current_seq=live_current_seq,
                deadline_last_reached_seq=reached_seq,
                dynamic_update_total_ms=total_ms,
                dynamic_r_lat=float(candidate['lat']),
                dynamic_r_lon=float(candidate['lon']),
                dynamic_r_alt=float(candidate['alt']),
                verification_reason=verification_reason,
                **self._gps_snapshot_fields(),
            )
            return True

    def _finish_dynamic_abort(self, reason, started):
        with self._dynamic_state_lock:
            deadline_reason, current_seq, reached_seq = (
                self._dynamic_deadline_evidence_locked()
            )
            state = self._dynamic_update_state
            cancel_reason = self._dynamic_worker_cancel_reason or reason
        event = (
            'dynamic_update_too_late'
            if state == 'TOO_LATE' or deadline_reason in {
                'r_reached', 'mission_current_r'
            }
            else 'dynamic_update_cancelled'
        )
        self._publish_aburcd_event(
            event,
            failure_reason=reason,
            r_source='SAFE',
            dynamic_worker_cancel_reason=cancel_reason,
            dynamic_update_total_ms=(time.monotonic() - started) * 1000.0,
            deadline_current_seq=current_seq,
            deadline_last_reached_seq=reached_seq,
            **self._gps_snapshot_fields(),
        )

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
        """Build prefix + A/U/R-safe/[release]/D + configured suffix."""
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
        self.composite_seq_u = self.composite_seq_a + 1
        self.composite_seq_r = self.composite_seq_u + 1
        if self.release_command_enabled:
            self.composite_seq_release = self.composite_seq_r + 1
            self.composite_seq_d = self.composite_seq_release + 1
        else:
            self.composite_seq_release = None
            self.composite_seq_d = self.composite_seq_r + 1
        self.dynamic_indices = {
            'A': self.composite_seq_a,
            'U': self.composite_seq_u,
            'R': self.composite_seq_r,
            'RELEASE': self.composite_seq_release,
            'D': self.composite_seq_d,
            'RESUME': self.composite_seq_d + 1,
        }

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
                abcdr['U']['lat'],
                abcdr['U']['lon'],
                altitude_m,
                self.u_acceptance_radius_m,
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
        self.composite_safe_r_waypoint = self.clone_waypoint(
            composite[self.composite_seq_r]
        )
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
        if seq == self.composite_seq_u:
            return 'U'
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

    async def call_service_async(
        self, client, request, timeout_sec, name, abort_check=None, before_send=None
    ):
        if not client.wait_for_service(
            timeout_sec=self.service_availability_timeout_sec
        ):
            raise RuntimeError(f'{name} service not available.')
        if before_send is not None:
            before_send()
        future = client.call_async(request)
        start_time = time.monotonic()
        while rclpy.ok() and not future.done():
            if abort_check is not None:
                abort_reason = abort_check()
                if abort_reason:
                    future.cancel()
                    raise DynamicUpdateAborted(f'{name}:{abort_reason}')
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

    async def push_mission_async(self, waypoints, retry=True, started_monotonic=None):
        if not self.allow_mission_upload:
            raise RuntimeError(
                'Mission upload is disabled by allow_mission_upload.'
            )

        async def operation():
            req = WaypointPush.Request()
            req.start_index = 0
            req.waypoints = waypoints
            result = await self.call_service_async(
                self.mission_push_client,
                req,
                self.service_timeout_sec,
                'WaypointPush',
                before_send=(
                    lambda: self._require_dynamic_operation_allowed(
                        started_monotonic, 'full_push_service_dispatch')
                ) if started_monotonic is not None else None,
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

        if not retry:
            return await operation()
        return await self._retry_mission_service('WaypointPush', operation)

    async def pull_mission_async(
        self, timeout_sec=None, abort_check=None
    ):
        req = WaypointPull.Request()
        generation_before = getattr(self, '_waypoint_list_generation', 0)
        effective_timeout = (
            self.service_timeout_sec
            if timeout_sec is None else max(0.05, float(timeout_sec))
        )
        result = await self.call_service_async(
            self.mission_pull_client,
            req,
            effective_timeout,
            'WaypointPull',
            abort_check=abort_check,
        )
        if not result.success:
            raise RuntimeError('WaypointPull rejected.')
        expected_count = int(result.wp_received)
        start_time = time.time()
        while time.time() - start_time < effective_timeout:
            if abort_check is not None:
                abort_reason = abort_check()
                if abort_reason:
                    raise DynamicUpdateAborted(
                        f'WaypointPullWait:{abort_reason}'
                    )
            if (
                self.current_waypoints is not None
                and getattr(self, '_waypoint_list_generation', 0)
                > generation_before
            ):
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

    async def set_current_mission_item_async(self, seq, dynamic_started=None):
        if not self.allow_mission_upload:
            raise RuntimeError('Mission set-current is disabled by allow_mission_upload.')
        req = WaypointSetCurrent.Request()
        req.wp_seq = int(seq)

        def guard_recovery():
            with self._dynamic_state_lock:
                reason = self._dynamic_abort_reason_locked(dynamic_started)
                if reason:
                    raise DynamicUpdateAborted(f'set_current_dispatch:{reason}')
                current = self._current_mission_seq()
                if current is None:
                    raise DynamicUpdateAborted('recovery current sequence unavailable')
                if current >= req.wp_seq:
                    raise DynamicProgressRecovered()
                req.wp_seq = max(req.wp_seq, current, self.last_reached_seq + 1)
                if not self.last_reached_seq < req.wp_seq < self.dynamic_indices['R']:
                    raise DynamicUpdateAborted('no safe resume sequence before R')

        try:
            result = await self.call_service_async(
                self.mission_set_current_client,
                req,
                self.service_timeout_sec,
                'WaypointSetCurrent',
                before_send=guard_recovery if dynamic_started is not None else None,
            )
        except DynamicProgressRecovered:
            return None
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
            'U': self.u_acceptance_radius_m,
            'R': self.c_acceptance_radius_m,
            'C': self.c_acceptance_radius_m,
            'D': self.d_acceptance_radius_m,
        }
        for item in ('A', 'B', 'U', 'R', 'C', 'D'):
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
            order = 'original_prefix,A,U,R,DO_SET_SERVO,D,original_suffix'
        else:
            order = 'original_prefix,A,U,R,D,original_suffix'
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
        attack_segment_item_count = 5 if self.release_command_enabled else 4
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
            f'b_virtual=true u_seq={self.composite_seq_u} '
            f'r_seq={self.composite_seq_r} '
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
        self._last_mission_current_seq = None
        self._b_previous_signed_m = None
        self._b_crossing_triggered = False
        self._dynamic_update_started = False
        self._dynamic_update_state = 'IDLE'
        self._dynamic_update_cancelled = False
        self._dynamic_worker_cancel_reason = None
        self._dynamic_metrics = {}
        self._second_full_push_attempted = False
        self.b_frozen_snapshot = None
        self._dynamic_update_verified = False
        self._dynamic_update_verified_monotonic = None
        self._dynamic_update_verified_gps = None
        self._dynamic_update_result = 'NOT_OBSERVED'
        self._r_active_before_verify_reported = False
        self._dynamic_prediction_commit = None
        self._dynamic_prediction_shadow = None
        self.composite_route_points = {
            key: dict(value)
            for key, value in abcdr.items()
            if isinstance(value, dict)
        }
        self.composite_route_points['heading_deg'] = float(abcdr['heading_deg'])
        self.composite_route_points['reverse_heading_deg'] = float(
            abcdr['reverse_heading_deg']
        )

        self._safe_composite_mission = None
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
        self._safe_composite_mission = tuple(copy.deepcopy(composite_waypoints))
        self.composite_completion_reported = False
        self.composite_total_count = len(composite_waypoints)
        self.composite_final_seq = self.composite_seq_d
        self.composite_expected_waypoints = [
            self.clone_waypoint(wp) for wp in composite_waypoints
        ]
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
            u_seq=self.composite_seq_u,
            r_seq=self.composite_seq_r,
            release_seq=self.composite_seq_release,
            release_command_enabled=self.release_command_enabled,
            c_virtual=True,
            d_seq=self.composite_seq_d,
            dynamic_indices=dict(self.dynamic_indices),
            target_lat=abcdr['C']['lat'],
            target_lon=abcdr['C']['lon'],
            heading_deg=abcdr['heading_deg'],
            points={
                key: dict(abcdr[key])
                for key in ('A', 'B', 'U', 'R', 'C', 'D')
            },
        )
        self.get_logger().info(
            self._prefix('FCU')
            + f' composite mission pull-back verified count={len(composite_waypoints)}'
        )
        self.get_logger().info(
            '[DYNAMIC_MISSION] '
            + ' '.join(
                f'{name}={seq}'
                for name, seq in self.dynamic_indices.items()
            )
            + ' B=virtual C=virtual'
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
            f'a_seq={a_seq}, u_seq={self.composite_seq_u}, '
            f'r_seq={self.composite_seq_r}, start_seq={start_seq}, '
            f'original_count={original_count}, total_count={len(composite_waypoints)}'
        )


class DynamicUpdateAborted(RuntimeError):
    """A dynamic update may no longer start or commit."""


class DynamicProgressRecovered(RuntimeError):
    """Fresh telemetry makes a pending SetCurrent unnecessary."""


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
