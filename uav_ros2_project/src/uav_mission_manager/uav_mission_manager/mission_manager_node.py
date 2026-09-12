import json
import math
import struct
import threading
import time
import uuid
from enum import Enum

import rclpy
from mavros_msgs.msg import (
    EstimatorStatus,
    GPSRAW,
    Mavlink,
    State,
    SysStatus,
    WaypointList,
)
from mavros_msgs.srv import VehicleInfoGet
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from uav_interfaces.msg import TargetCommand
from uav_interfaces.srv import GoToGlobal


class MissionState(str, Enum):
    BOOT = 'BOOT'
    WAIT_FCU = 'WAIT_FCU'
    STANDBY = 'STANDBY'
    EXECUTING = 'EXECUTING'
    MISSION_COMPLETE = 'MISSION_COMPLETE'
    SAFE = 'SAFE'


class RuntimeHealthLevel(str, Enum):
    """Companion-side runtime observation severity."""

    OK = 'OK'
    WARNING = 'WARNING'
    DEGRADED = 'DEGRADED'
    CRITICAL = 'CRITICAL'


class MissionManagerNode(Node):
    """
    Safety-gated mission coordinator.

    Once the expected FCU identity and state link are stable, the manager enters
    STANDBY. A target received in STANDBY must pass the attack-start gate before
    it can authorize a dynamic mission. Runtime telemetry observations are
    reported separately from the primary mission state.
    """

    LEGAL_TRANSITIONS = {
        MissionState.BOOT: {MissionState.WAIT_FCU, MissionState.SAFE},
        MissionState.WAIT_FCU: {MissionState.STANDBY, MissionState.SAFE},
        MissionState.STANDBY: {
            MissionState.EXECUTING,
            MissionState.WAIT_FCU,
            MissionState.SAFE,
        },
        MissionState.EXECUTING: {
            MissionState.MISSION_COMPLETE,
            MissionState.SAFE,
        },
        MissionState.MISSION_COMPLETE: {
            MissionState.STANDBY,
            MissionState.SAFE,
        },
        MissionState.SAFE: {MissionState.WAIT_FCU},
    }

    def __init__(self):
        super().__init__('mission_manager_node')
        self._lock = threading.RLock()
        self.boot_monotonic_time = time.monotonic()
        self.boot_wall_time = time.time()
        self.boot_session_id = uuid.uuid4().hex

        self._declare_parameters()
        self.parameters_loaded = True
        self.parameters_valid, parameter_reason = self._load_and_validate_parameters()
        self.configuration_ready = self.parameters_loaded and self.parameters_valid

        self.state = MissionState.BOOT
        self.last_transition_reason = 'process_started'
        self.safe_reason = 'none'
        self.busy = False
        self.execution_allowed = False
        # Process-lifetime latch: only a node restart permits another attack.
        self.attack_executed = False
        self.target_valid = False
        self.active_target = None
        self.target_counter = 0
        self.execution_generation = 0
        self.current_mission_seq = None
        self.current_mission_count = 0
        self.last_mission_waypoints_time = None

        self.fcu_connected = False
        self.fcu_armed = False
        self.fcu_mode = 'UNKNOWN'
        self.fcu_system_status = 0
        self.actual_system_id = None
        self.actual_component_id = None
        self.last_heartbeat_time = None
        self.last_fcu_state_time = None
        self.last_position_time = None
        self.last_gps_time = None
        self.last_ekf_time = None
        self.ekf_source = 'none'
        self.last_sensor_time = None
        self.heartbeat_sequence_start = None
        self.heartbeat_count = 0
        self.position_healthy = False
        self.gps_healthy = False
        self.gps_fix_type = 0
        self.gps_satellites = 0
        self.gps_h_acc_m = float('inf')
        self.ekf_healthy = False
        self.sensor_health = False
        self.optional_fcu_sensor_warning = False
        self.external_health = {}
        self.external_health_time = {}
        self._vehicle_info_future = None
        self._vehicle_info_request_time = None
        self.last_vehicle_info_time = None
        self.runtime_fault_since = {}
        self.runtime_health_level = RuntimeHealthLevel.OK
        self.runtime_faults = []
        self.runtime_critical_faults = []

        self._create_ros_interfaces()

        if self.configuration_ready:
            self.transition_to(MissionState.WAIT_FCU, 'boot_initialization_complete')
        else:
            self.enter_safe(f'parameter_validation_failed:{parameter_reason}')

        self.get_logger().info(
            self._prefix()
            + f' manager started boot_session_id={self.boot_session_id} '
            'execution_allowed=false attack_executed=false target_valid=false '
            f'parameters_valid={self._bool(self.parameters_valid)}'
        )

    def _declare_parameters(self):
        self.declare_parameter('acceptance_radius_m', 5.0)
        self.declare_parameter('task_id_topic', '/mission/active_task_id')
        self.declare_parameter('state_topic', '/mission/safety_state')
        self.declare_parameter('manager_state_topic', '/mission_manager/state')
        self.declare_parameter('target_valid_topic', '/mission_manager/target_valid')
        self.declare_parameter('diagnostic_topic', '/mission/safety_status')
        self.declare_parameter('target_topic', '/vision/target_command')

        self.declare_parameter('expected_system_id', 1)
        self.declare_parameter('expected_component_id', 1)
        self.declare_parameter('heartbeat_timeout_sec', 2.0)
        self.declare_parameter('heartbeat_required_count', 5)
        self.declare_parameter('heartbeat_stable_duration_sec', 3.0)
        self.declare_parameter('fcu_state_timeout_sec', 2.0)
        self.declare_parameter('position_timeout_sec', 2.0)
        self.declare_parameter('gps_timeout_sec', 2.0)
        self.declare_parameter('ekf_timeout_sec', 2.0)
        self.declare_parameter('sensor_timeout_sec', 3.0)
        self.declare_parameter('vehicle_info_timeout_sec', 3.0)
        self.declare_parameter('vehicle_info_refresh_sec', 5.0)
        self.declare_parameter('health_check_period_sec', 0.2)
        self.declare_parameter('runtime_degraded_after_sec', 5.0)

        self.declare_parameter('gps_required', True)
        self.declare_parameter('gps_min_fix_type', 3)
        self.declare_parameter('gps_min_satellites', 6)
        self.declare_parameter('gps_max_horizontal_accuracy_m', 10.0)
        self.declare_parameter('position_required', True)
        self.declare_parameter('ekf_required', True)
        self.declare_parameter('required_sensor_mask', 0)
        self.declare_parameter('optional_sensor_mask', 0)
        # A non-empty string default establishes ROS's STRING_ARRAY type. ROS 2
        # Humble cannot infer the type of a YAML `[]` override, so the config uses
        # the same blank sentinel and loading below filters it out.
        self.declare_parameter('required_external_health_topics', [''])
        self.declare_parameter('optional_external_health_topics', [''])
        self.declare_parameter('serious_system_statuses', [5, 6, 7, 8])

        self.declare_parameter('target_max_age_sec', 15.0)
        self.declare_parameter('target_min_latitude', -90.0)
        self.declare_parameter('target_max_latitude', 90.0)
        self.declare_parameter('target_min_longitude', -180.0)
        self.declare_parameter('target_max_longitude', 180.0)
        self.declare_parameter('insert_wp_index', 5)
        self.declare_parameter('reject_target_at_or_after_insert_wp', True)

    def _load_and_validate_parameters(self):
        try:
            self.acceptance_radius_m = float(
                self.get_parameter('acceptance_radius_m').value
            )
            self.expected_system_id = int(self.get_parameter('expected_system_id').value)
            self.expected_component_id = int(
                self.get_parameter('expected_component_id').value
            )
            self.heartbeat_timeout_sec = float(
                self.get_parameter('heartbeat_timeout_sec').value
            )
            self.heartbeat_required_count = int(
                self.get_parameter('heartbeat_required_count').value
            )
            self.heartbeat_stable_duration_sec = float(
                self.get_parameter('heartbeat_stable_duration_sec').value
            )
            self.fcu_state_timeout_sec = float(
                self.get_parameter('fcu_state_timeout_sec').value
            )
            self.position_timeout_sec = float(
                self.get_parameter('position_timeout_sec').value
            )
            self.gps_timeout_sec = float(self.get_parameter('gps_timeout_sec').value)
            self.ekf_timeout_sec = float(self.get_parameter('ekf_timeout_sec').value)
            self.sensor_timeout_sec = float(
                self.get_parameter('sensor_timeout_sec').value
            )
            self.vehicle_info_timeout_sec = float(
                self.get_parameter('vehicle_info_timeout_sec').value
            )
            self.vehicle_info_refresh_sec = float(
                self.get_parameter('vehicle_info_refresh_sec').value
            )
            self.health_check_period_sec = float(
                self.get_parameter('health_check_period_sec').value
            )
            self.runtime_degraded_after_sec = float(
                self.get_parameter('runtime_degraded_after_sec').value
            )
            self.gps_required = bool(self.get_parameter('gps_required').value)
            self.gps_min_fix_type = int(self.get_parameter('gps_min_fix_type').value)
            self.gps_min_satellites = int(
                self.get_parameter('gps_min_satellites').value
            )
            self.gps_max_horizontal_accuracy_m = float(
                self.get_parameter('gps_max_horizontal_accuracy_m').value
            )
            self.position_required = bool(self.get_parameter('position_required').value)
            self.ekf_required = bool(self.get_parameter('ekf_required').value)
            self.required_sensor_mask = int(
                self.get_parameter('required_sensor_mask').value
            )
            self.optional_sensor_mask = int(
                self.get_parameter('optional_sensor_mask').value
            )
            self.required_external_health_topics = [
                str(value)
                for value in self.get_parameter(
                    'required_external_health_topics'
                ).value
                if str(value).strip()
            ]
            self.optional_external_health_topics = [
                str(value)
                for value in self.get_parameter(
                    'optional_external_health_topics'
                ).value
                if str(value).strip()
            ]
            self.serious_system_statuses = {
                int(value)
                for value in self.get_parameter('serious_system_statuses').value
            }
            self.target_max_age_sec = float(
                self.get_parameter('target_max_age_sec').value
            )
            self.target_min_latitude = float(
                self.get_parameter('target_min_latitude').value
            )
            self.target_max_latitude = float(
                self.get_parameter('target_max_latitude').value
            )
            self.target_min_longitude = float(
                self.get_parameter('target_min_longitude').value
            )
            self.target_max_longitude = float(
                self.get_parameter('target_max_longitude').value
            )
            self.insert_wp_index = int(self.get_parameter('insert_wp_index').value)
            self.reject_target_at_or_after_insert_wp = self.parameter_as_bool(
                self.get_parameter('reject_target_at_or_after_insert_wp').value
            )
        except (TypeError, ValueError) as exc:
            return False, f'parameter_type_error:{exc}'

        positive_values = {
            'acceptance_radius_m': self.acceptance_radius_m,
            'heartbeat_timeout_sec': self.heartbeat_timeout_sec,
            'heartbeat_stable_duration_sec': self.heartbeat_stable_duration_sec,
            'fcu_state_timeout_sec': self.fcu_state_timeout_sec,
            'position_timeout_sec': self.position_timeout_sec,
            'gps_timeout_sec': self.gps_timeout_sec,
            'ekf_timeout_sec': self.ekf_timeout_sec,
            'sensor_timeout_sec': self.sensor_timeout_sec,
            'vehicle_info_timeout_sec': self.vehicle_info_timeout_sec,
            'vehicle_info_refresh_sec': self.vehicle_info_refresh_sec,
            'health_check_period_sec': self.health_check_period_sec,
            'runtime_degraded_after_sec': self.runtime_degraded_after_sec,
            'target_max_age_sec': self.target_max_age_sec,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                return False, f'{name}_must_be_positive'
        if not 1 <= self.expected_system_id <= 255:
            return False, 'expected_system_id_out_of_range'
        if not 1 <= self.expected_component_id <= 255:
            return False, 'expected_component_id_out_of_range'
        if self.heartbeat_required_count < 2:
            return False, 'heartbeat_required_count_must_be_at_least_2'
        if not 0 <= self.gps_min_fix_type <= 8:
            return False, 'gps_min_fix_type_out_of_range'
        if not 0 <= self.gps_min_satellites <= 255:
            return False, 'gps_min_satellites_out_of_range'
        if self.gps_max_horizontal_accuracy_m <= 0.0:
            return False, 'gps_max_horizontal_accuracy_m_must_be_positive'
        if self.insert_wp_index <= 0:
            return False, 'insert_wp_index_must_be_positive'
        all_external_topics = (
            self.required_external_health_topics
            + self.optional_external_health_topics
        )
        if any(not topic.startswith('/') for topic in all_external_topics):
            return False, 'external_health_topics_must_be_absolute'
        if len(set(all_external_topics)) != len(all_external_topics):
            return False, 'external_health_topics_must_be_unique'
        if not (-90.0 <= self.target_min_latitude <= self.target_max_latitude <= 90.0):
            return False, 'target_latitude_bounds_invalid'
        if not (
            -180.0
            <= self.target_min_longitude
            <= self.target_max_longitude
            <= 180.0
        ):
            return False, 'target_longitude_bounds_invalid'
        return True, 'parameters_valid'

    def _create_ros_interfaces(self):
        task_qos = QoSProfile(depth=1)
        task_qos.reliability = ReliabilityPolicy.RELIABLE
        task_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.task_id_publisher = self.create_publisher(
            String,
            str(self.get_parameter('task_id_topic').value),
            task_qos,
        )
        self.state_publisher = self.create_publisher(
            String,
            str(self.get_parameter('state_topic').value),
            task_qos,
        )
        self.manager_state_publisher = self.create_publisher(
            String,
            str(self.get_parameter('manager_state_topic').value),
            task_qos,
        )
        self.target_valid_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('target_valid_topic').value),
            task_qos,
        )
        self.diagnostic_publisher = self.create_publisher(
            String,
            str(self.get_parameter('diagnostic_topic').value),
            10,
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            TargetCommand,
            str(self.get_parameter('target_topic').value),
            self.target_callback,
            10,
        )
        self.create_subscription(State, '/mavros/state', self.fcu_state_callback, 10)
        self.create_subscription(
            WaypointList,
            '/mavros/mission/waypoints',
            self.mission_waypoints_callback,
            sensor_qos,
        )
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.position_callback,
            sensor_qos,
        )
        self.create_subscription(
            GPSRAW,
            '/mavros/gpsstatus/gps1/raw',
            self.gps_callback,
            sensor_qos,
        )
        self.create_subscription(
            EstimatorStatus,
            '/mavros/estimator_status',
            self.ekf_callback,
            sensor_qos,
        )
        # ArduPilot publishes EKF_STATUS_REPORT (MAVLink message 193), while
        # MAVROS 2.14 only converts ESTIMATOR_STATUS (message 230) to the ROS
        # topic above.  Observe the MAVROS router's read-only raw stream as a
        # compatibility fallback; this path never writes to the flight
        # controller.
        self.create_subscription(
            Mavlink,
            '/uas1/mavlink_source',
            self.mavlink_ekf_callback,
            sensor_qos,
        )
        self.create_subscription(
            SysStatus,
            '/mavros/sys_status',
            self.sensor_callback,
            sensor_qos,
        )
        self.create_subscription(
            String,
            '/fcu/composite_mission_complete',
            self.composite_mission_complete_callback,
            10,
        )
        for topic in (
            self.required_external_health_topics
            + self.optional_external_health_topics
        ):
            self.external_health[topic] = False
            self.external_health_time[topic] = None
            self.create_subscription(
                Bool,
                topic,
                lambda msg, health_topic=topic: self.external_health_callback(
                    health_topic, msg
                ),
                10,
            )

        self.disable_service = self.create_service(
            Trigger, '/mission/disable', self.disable_callback
        )

        self.goto_client = self.create_client(GoToGlobal, '/fcu/goto_global')
        self.vehicle_info_client = self.create_client(
            VehicleInfoGet, '/mavros/vehicle_info_get'
        )

        self.create_timer(self.health_check_period_sec, self.health_timer_callback)
        self.create_timer(2.0, self.print_state)

    @staticmethod
    def _bool(value):
        return str(bool(value)).lower()

    @staticmethod
    def parameter_as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in ('true', '1', 'yes', 'on'):
                return True
            if normalized in ('false', '0', 'no', 'off'):
                return False
            raise ValueError(f'invalid boolean value: {value}')
        return bool(value)

    @staticmethod
    def valid_lat_lon(latitude, longitude):
        return (
            math.isfinite(latitude)
            and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0
            and -180.0 <= longitude <= 180.0
        )

    @staticmethod
    def valid_heading(heading_deg):
        return math.isfinite(heading_deg)

    @staticmethod
    def is_data_fresh(last_update_time, timeout_sec, now=None):
        if last_update_time is None:
            return False
        current = time.monotonic() if now is None else now
        return 0.0 <= current - last_update_time <= timeout_sec

    @staticmethod
    def data_age(last_update_time, now=None):
        if last_update_time is None:
            return None
        current = time.monotonic() if now is None else now
        return max(0.0, current - last_update_time)

    def _prefix(self, module='MISSION', state=None):
        task_id = self.active_target['id'] if self.active_target else 'none'
        selected_state = state or self.state.value
        if isinstance(selected_state, MissionState):
            selected_state = selected_state.value
        return f'[{module}][task={task_id}][state={selected_state}]'

    def transition_to(self, new_state, reason):
        with self._lock:
            if not isinstance(new_state, MissionState):
                new_state = MissionState(new_state)
            old_state = self.state
            if old_state == new_state:
                self.last_transition_reason = str(reason)
                if new_state == MissionState.SAFE:
                    self.safe_reason = str(reason)
                self._publish_state_locked()
                return True
            if new_state not in self.LEGAL_TRANSITIONS.get(old_state, set()):
                self.get_logger().error(
                    self._prefix('SAFETY', old_state)
                    + f' illegal transition requested old={old_state.value} '
                    f'new={new_state.value} reason={reason}'
                )
                return False

            if new_state == MissionState.SAFE:
                self._clear_task_authority_locked()
                self.safe_reason = str(reason)
            elif new_state == MissionState.WAIT_FCU:
                self._clear_task_authority_locked()
            elif new_state == MissionState.STANDBY:
                self._clear_task_authority_locked()
                self.safe_reason = 'none'
            elif new_state == MissionState.EXECUTING:
                self.execution_allowed = True
                self.busy = True
            elif new_state == MissionState.MISSION_COMPLETE:
                self.execution_allowed = False
                self.busy = False

            self.state = new_state
            self.last_transition_reason = str(reason)
            self.get_logger().info(
                self._prefix('SAFETY')
                + f' state transition old={old_state.value} new={new_state.value} '
                f'reason={reason}'
            )
            self._publish_state_locked()
            return True

    def enter_safe(self, reason):
        with self._lock:
            self.get_logger().error(
                self._prefix('SAFETY', 'SAFE')
                + f' entering SAFE reason={reason} '
                'new_companion_control_blocked=true '
                'fcu_abort_available=false fcu_abort_executed=false '
                'active_fcu_mission_not_cancelled=true '
                'uploaded_fcu_mission_may_continue=true'
            )
            self.transition_to(MissionState.SAFE, reason)

    def _clear_task_authority_locked(self):
        self._invalidate_target_locked()
        self.execution_allowed = False
        self.busy = False
        self.execution_generation += 1
        self._publish_task_id_locked('none')

    def _invalidate_target_locked(self):
        self.active_target = None
        self.target_valid = False
        self._publish_target_valid_locked()

    def _publish_task_id_locked(self, task_id):
        message = String()
        message.data = str(task_id)
        self.task_id_publisher.publish(message)

    def _publish_state_locked(self):
        message = String()
        message.data = self.state.value
        self.state_publisher.publish(message)
        self.manager_state_publisher.publish(message)

    def _publish_target_valid_locked(self):
        message = Bool()
        message.data = bool(self.target_valid)
        self.target_valid_publisher.publish(message)

    def fcu_state_callback(self, msg):
        now = time.monotonic()
        with self._lock:
            previous_connected = self.fcu_connected
            self.fcu_connected = bool(msg.connected)
            self.fcu_armed = bool(msg.armed)
            self.fcu_mode = str(msg.mode or 'UNKNOWN')
            self.fcu_system_status = int(msg.system_status)
            self.last_fcu_state_time = now

            if self.fcu_connected:
                if (
                    self.last_heartbeat_time is None
                    or now - self.last_heartbeat_time > self.heartbeat_timeout_sec
                ):
                    self.heartbeat_count = 0
                    self.heartbeat_sequence_start = now
                if self.heartbeat_sequence_start is None:
                    self.heartbeat_sequence_start = now
                self.heartbeat_count += 1
                self.last_heartbeat_time = now
            else:
                self._reset_fcu_identity_locked()
                self._invalidate_mission_waypoints_locked()

            if previous_connected and not self.fcu_connected:
                self.get_logger().warning(
                    self._prefix('SAFETY') + ' MAVROS disconnected'
                )

    def mission_waypoints_callback(self, msg):
        with self._lock:
            self.current_mission_seq = int(msg.current_seq)
            self.current_mission_count = len(msg.waypoints)
            self.last_mission_waypoints_time = time.monotonic()

    def _invalidate_mission_waypoints_locked(self):
        self.current_mission_seq = None
        self.current_mission_count = 0
        self.last_mission_waypoints_time = None

    def _reset_fcu_identity_locked(self):
        self.actual_system_id = None
        self.actual_component_id = None
        self.last_heartbeat_time = None
        self.heartbeat_sequence_start = None
        self.heartbeat_count = 0
        self._vehicle_info_future = None
        self._vehicle_info_request_time = None
        self.last_vehicle_info_time = None

    def position_callback(self, msg):
        now = time.monotonic()
        with self._lock:
            self.last_position_time = now
            self.position_healthy = (
                msg.status.status >= 0
                and self.valid_lat_lon(msg.latitude, msg.longitude)
                and not (abs(msg.latitude) < 1.0e-9 and abs(msg.longitude) < 1.0e-9)
            )

    def gps_callback(self, msg):
        now = time.monotonic()
        with self._lock:
            self.last_gps_time = now
            self.gps_fix_type = int(msg.fix_type)
            self.gps_satellites = int(msg.satellites_visible)
            self.gps_h_acc_m = (
                float(msg.h_acc) / 1000.0
                if int(msg.h_acc) < 0xFFFFFFFF
                else float('inf')
            )
            coordinates_valid = self.valid_lat_lon(msg.lat / 1.0e7, msg.lon / 1.0e7)
            self.gps_healthy = (
                self.gps_fix_type >= self.gps_min_fix_type
                and self.gps_satellites >= self.gps_min_satellites
                and self.gps_h_acc_m <= self.gps_max_horizontal_accuracy_m
                and coordinates_valid
            )

    def ekf_callback(self, msg):
        with self._lock:
            self.last_ekf_time = time.monotonic()
            self.ekf_source = 'mavros_estimator_status_230'
            horizontal_valid = (
                bool(msg.pos_horiz_abs_status_flag)
                or bool(msg.pos_horiz_rel_status_flag)
            )
            vertical_valid = (
                bool(msg.pos_vert_abs_status_flag)
                or bool(msg.pos_vert_agl_status_flag)
            )
            self.ekf_healthy = (
                bool(msg.attitude_status_flag)
                and horizontal_valid
                and vertical_valid
                and not bool(msg.gps_glitch_status_flag)
                and not bool(msg.accel_error_status_flag)
            )

    def mavlink_ekf_callback(self, msg):
        """Consume ArduPilot EKF_STATUS_REPORT from MAVROS' raw RX stream."""
        if int(msg.msgid) != 193 or int(msg.sysid) != self.expected_system_id:
            return
        if int(msg.len) < 22 or len(msg.payload64) < 3:
            return

        try:
            payload = b''.join(
                struct.pack('<Q', int(word)) for word in msg.payload64
            )[:int(msg.len)]
            flags = struct.unpack_from('<H', payload, 20)[0]
        except (OverflowError, struct.error, TypeError, ValueError):
            return

        # MAV_EKF_STATUS_REPORT_FLAGS from ardupilotmega.xml.
        attitude_valid = bool(flags & 0x01)
        horizontal_valid = bool(flags & (0x08 | 0x10))
        vertical_valid = bool(flags & (0x20 | 0x40))
        constant_position_mode = bool(flags & 0x80)
        uninitialized = bool(flags & 0x400)

        with self._lock:
            self.last_ekf_time = time.monotonic()
            self.ekf_source = 'ardupilot_ekf_status_report_193'
            self.ekf_healthy = (
                attitude_valid
                and horizontal_valid
                and vertical_valid
                and not constant_position_mode
                and not uninitialized
            )

    def sensor_callback(self, msg):
        with self._lock:
            self.last_sensor_time = time.monotonic()
            required = self.required_sensor_mask
            required_present = (int(msg.sensors_present) & required) == required
            required_enabled = (int(msg.sensors_enabled) & required) == required
            required_healthy = (int(msg.sensors_health) & required) == required
            self.sensor_health = (
                required_present and required_enabled and required_healthy
            )

            optional = self.optional_sensor_mask
            self.optional_fcu_sensor_warning = bool(
                optional and (int(msg.sensors_health) & optional) != optional
            )

    def external_health_callback(self, topic, msg):
        """Adapter for camera, payload, rangefinder, or communication health."""
        with self._lock:
            self.external_health[topic] = bool(msg.data)
            self.external_health_time[topic] = time.monotonic()

    def _request_vehicle_info_locked(self, now):
        if not self.fcu_connected:
            return
        if (
            self.actual_system_id is not None
            and self.last_vehicle_info_time is not None
            and now - self.last_vehicle_info_time < self.vehicle_info_refresh_sec
        ):
            return
        if self._vehicle_info_future is not None:
            if (
                self._vehicle_info_request_time is not None
                and now - self._vehicle_info_request_time
                > self.vehicle_info_timeout_sec
            ):
                self._vehicle_info_future = None
                self._vehicle_info_request_time = None
            return
        if not self.vehicle_info_client.service_is_ready():
            return
        request = VehicleInfoGet.Request()
        request.sysid = 0
        request.compid = 0
        request.get_all = False
        self._vehicle_info_request_time = now
        self._vehicle_info_future = self.vehicle_info_client.call_async(request)
        self._vehicle_info_future.add_done_callback(self.vehicle_info_done_callback)

    def vehicle_info_done_callback(self, future):
        with self._lock:
            if future is not self._vehicle_info_future:
                return
            self._vehicle_info_future = None
            self._vehicle_info_request_time = None
            try:
                result = future.result()
            except Exception as exc:
                self.get_logger().warning(
                    self._prefix('SAFETY')
                    + f' vehicle info request failed reason={exc!r}'
                )
                return
            if not result.success or not result.vehicles:
                return
            received_system_id = int(result.vehicles[0].sysid)
            received_component_id = int(result.vehicles[0].compid)
            if self.actual_system_id not in (None, received_system_id):
                previous_system_id = self.actual_system_id
                self.actual_system_id = received_system_id
                self.actual_component_id = received_component_id
                self.enter_safe(
                    f'fcu_system_id_changed:{previous_system_id}'
                    f'->{received_system_id}'
                )
                return
            if self.actual_component_id not in (None, received_component_id):
                previous_component_id = self.actual_component_id
                self.actual_system_id = received_system_id
                self.actual_component_id = received_component_id
                self.enter_safe(
                    'fcu_component_id_changed:'
                    f'{previous_component_id}->{received_component_id}'
                )
                return
            self.actual_system_id = received_system_id
            self.actual_component_id = received_component_id
            self.last_vehicle_info_time = time.monotonic()

    def _fcu_identity_confirmed_locked(self):
        return (
            self.actual_system_id == self.expected_system_id
            and self.actual_component_id == self.expected_component_id
        )

    def _fcu_state_link_stable_locked(self, now):
        if not self.fcu_connected:
            return False
        if not self.is_data_fresh(
            self.last_fcu_state_time, self.fcu_state_timeout_sec, now
        ):
            return False
        if self.heartbeat_count < self.heartbeat_required_count:
            return False
        if self.heartbeat_sequence_start is None:
            return False
        return now - self.heartbeat_sequence_start >= self.heartbeat_stable_duration_sec

    def _attack_start_blockers_locked(self, now):
        blockers = []
        if not self.configuration_ready:
            blockers.append('configuration_not_ready')

        fcu_state_fresh = self.is_data_fresh(
            self.last_fcu_state_time, self.fcu_state_timeout_sec, now
        )
        if not self.fcu_connected:
            blockers.append('fcu_not_connected')
        elif not fcu_state_fresh:
            blockers.append('fcu_state_data_stale')
        elif not self._fcu_state_link_stable_locked(now):
            blockers.append('fcu_state_link_not_stable')

        if self.actual_system_id is None or self.actual_component_id is None:
            blockers.append('fcu_identity_not_confirmed')
        else:
            if self.actual_system_id != self.expected_system_id:
                blockers.append('system_id_mismatch')
            if self.actual_component_id != self.expected_component_id:
                blockers.append('component_id_mismatch')

        if not self.fcu_armed:
            blockers.append('aircraft_not_armed')
        if self.fcu_mode.strip().upper() != 'AUTO':
            blockers.append('flight_mode_not_auto')

        if self.position_required:
            if not self.is_data_fresh(
                self.last_position_time, self.position_timeout_sec, now
            ):
                blockers.append('position_data_stale')
            elif not self.position_healthy:
                blockers.append('position_invalid')

        if self.gps_required:
            if not self.is_data_fresh(
                self.last_gps_time, self.gps_timeout_sec, now
            ):
                blockers.append('gps_data_stale')
            elif not self.gps_healthy:
                blockers.append('gps_health_bad')

        if self.ekf_required:
            if not self.is_data_fresh(
                self.last_ekf_time, self.ekf_timeout_sec, now
            ):
                blockers.append('ekf_data_stale')
            elif not self.ekf_healthy:
                blockers.append('ekf_health_bad')

        if (
            fcu_state_fresh
            and self.fcu_system_status in self.serious_system_statuses
        ):
            blockers.append(
                f'fcu_serious_system_status_{self.fcu_system_status}'
            )

        if self.required_sensor_mask:
            if not self.is_data_fresh(
                self.last_sensor_time, self.sensor_timeout_sec, now
            ):
                blockers.append('required_sensor_data_stale')
            elif not self.sensor_health:
                blockers.append('required_sensor_health_bad')

        for topic in self.required_external_health_topics:
            if not self.is_data_fresh(
                self.external_health_time.get(topic),
                self.sensor_timeout_sec,
                now,
            ):
                blockers.append(f'external_health_data_stale:{topic}')
            elif not self.external_health.get(topic, False):
                blockers.append(f'external_health_bad:{topic}')

        insert_window_blocker = self._target_insert_window_blocker_locked(now)
        if insert_window_blocker:
            blockers.append(f'insert_window_{insert_window_blocker}')

        if not self.goto_client.service_is_ready():
            blockers.append('goto_global_service_not_ready')
        return blockers

    def _runtime_health_faults_locked(self, now):
        warnings = []
        critical = []
        fcu_state_fresh = self.is_data_fresh(
            self.last_fcu_state_time, self.fcu_state_timeout_sec, now
        )

        if not self.configuration_ready:
            critical.append('configuration_not_ready')
        if self.actual_system_id not in (None, self.expected_system_id):
            critical.append('system_id_mismatch')
        if self.actual_component_id not in (None, self.expected_component_id):
            critical.append('component_id_mismatch')

        if not self.fcu_connected:
            warnings.append('fcu_not_connected')
        elif not fcu_state_fresh:
            warnings.append('fcu_state_data_stale')
        if not self._fcu_identity_confirmed_locked():
            warnings.append('fcu_identity_not_confirmed')
        if (
            fcu_state_fresh
            and self.fcu_system_status in self.serious_system_statuses
        ):
            critical.append(
                f'fcu_serious_system_status_{self.fcu_system_status}'
            )

        vehicle_info_max_age = (
            self.vehicle_info_refresh_sec + self.vehicle_info_timeout_sec
        )
        if self.fcu_connected and not self.is_data_fresh(
            self.last_vehicle_info_time, vehicle_info_max_age, now
        ):
            warnings.append('vehicle_info_data_stale')

        if self.position_required:
            if not self.is_data_fresh(
                self.last_position_time, self.position_timeout_sec, now
            ):
                warnings.append('position_data_stale')
            elif not self.position_healthy:
                warnings.append('position_invalid')

        if self.gps_required:
            if not self.is_data_fresh(
                self.last_gps_time, self.gps_timeout_sec, now
            ):
                warnings.append('gps_data_stale')
            elif not self.gps_healthy:
                warnings.append('gps_health_bad')

        if self.ekf_required:
            if not self.is_data_fresh(
                self.last_ekf_time, self.ekf_timeout_sec, now
            ):
                warnings.append('ekf_data_stale')
            elif not self.ekf_healthy:
                warnings.append('ekf_health_bad')

        if not self.is_data_fresh(
            self.last_sensor_time, self.sensor_timeout_sec, now
        ):
            warnings.append('sensor_data_stale')
        else:
            if self.required_sensor_mask and not self.sensor_health:
                warnings.append('required_sensor_health_bad')
            if self.optional_fcu_sensor_warning:
                warnings.append('optional_sensor_health_bad')

        for topic in (
            self.required_external_health_topics
            + self.optional_external_health_topics
        ):
            if not self.is_data_fresh(
                self.external_health_time.get(topic),
                self.sensor_timeout_sec,
                now,
            ):
                warnings.append(f'external_health_data_stale:{topic}')
            elif not self.external_health.get(topic, False):
                warnings.append(f'external_health_bad:{topic}')
        return warnings, critical

    def _update_runtime_health_locked(self, now):
        warnings, critical = self._runtime_health_faults_locked(now)
        faults = warnings + critical
        previous_faults = set(self.runtime_faults)
        current_faults = set(faults)

        for fault in faults:
            if fault not in self.runtime_fault_since:
                self.runtime_fault_since[fault] = now
                severity = (
                    RuntimeHealthLevel.CRITICAL.value
                    if fault in critical
                    else RuntimeHealthLevel.WARNING.value
                )
                log = (
                    self.get_logger().error
                    if fault in critical
                    else self.get_logger().warning
                )
                log(
                    self._prefix('RUNTIME_HEALTH')
                    + f' fault={fault} duration_s=0.000 severity={severity}'
                )

        for fault in sorted(previous_faults - current_faults):
            started = self.runtime_fault_since.pop(fault, now)
            self.get_logger().info(
                self._prefix('RUNTIME_HEALTH')
                + f' fault={fault} recovered=true '
                f'duration_s={max(0.0, now - started):.3f}'
            )

        if critical:
            level = RuntimeHealthLevel.CRITICAL
        elif faults and any(
            now - self.runtime_fault_since[fault]
            >= self.runtime_degraded_after_sec
            for fault in faults
        ):
            level = RuntimeHealthLevel.DEGRADED
        elif faults:
            level = RuntimeHealthLevel.WARNING
        else:
            level = RuntimeHealthLevel.OK

        if level != self.runtime_health_level:
            self.get_logger().warning(
                self._prefix('RUNTIME_HEALTH')
                + f' level_changed old={self.runtime_health_level.value} '
                f'new={level.value} faults={",".join(faults) or "none"}'
            )
        self.runtime_health_level = level
        self.runtime_faults = faults
        self.runtime_critical_faults = critical
        return level, faults, critical

    def health_timer_callback(self):
        now = time.monotonic()
        with self._lock:
            self._request_vehicle_info_locked(now)

            if (
                self.state == MissionState.STANDBY
                and self.active_target is not None
                and self.target_valid
            ):
                if now - self.active_target['received_monotonic'] > self.target_max_age_sec:
                    self.get_logger().warning(
                        self._prefix('SAFETY', 'REJECTED')
                        + ' target invalidated reason=target_expired'
                    )
                    self.target_valid = False
                    self._publish_target_valid_locked()

            if self.state == MissionState.WAIT_FCU:
                if not self.configuration_ready:
                    self.enter_safe('configuration_not_ready')
                elif self.actual_system_id not in (None, self.expected_system_id):
                    self.enter_safe(
                        f'system_id_mismatch:expected={self.expected_system_id},'
                        f'actual={self.actual_system_id}'
                    )
                elif self.actual_component_id not in (None, self.expected_component_id):
                    self.enter_safe(
                        'component_id_mismatch:'
                        f'expected={self.expected_component_id},'
                        f'actual={self.actual_component_id}'
                    )
                elif (
                    self._fcu_identity_confirmed_locked()
                    and self._fcu_state_link_stable_locked(now)
                ):
                    self.transition_to(
                        MissionState.STANDBY,
                        'fcu_identity_and_link_stable',
                    )
                return

            if self.state == MissionState.STANDBY:
                _, _, critical = self._update_runtime_health_locked(now)
                if critical:
                    self.enter_safe(
                        'standby_critical_health:' + ','.join(critical)
                    )
                elif (
                    not self.fcu_connected
                    or not self.is_data_fresh(
                        self.last_fcu_state_time,
                        self.fcu_state_timeout_sec,
                        now,
                    )
                ):
                    self.transition_to(
                        MissionState.WAIT_FCU,
                        'fcu_link_lost_reconnect_required',
                    )
                return

            if self.state == MissionState.EXECUTING:
                level, _, critical = self._update_runtime_health_locked(now)
                if level == RuntimeHealthLevel.CRITICAL:
                    self.get_logger().error(
                        self._prefix('SAFETY')
                        + ' CRITICAL detected '
                        f'faults={",".join(critical)} '
                        'mission_manager_control_blocked=true '
                        'fcu_abort_executed=false '
                        'uploaded_fcu_mission_may_continue=true'
                    )
                    self.enter_safe(
                        'runtime_critical_health:' + ','.join(critical)
                    )
                return

            if self.state == MissionState.SAFE:
                _, critical = self._runtime_health_faults_locked(now)
                if (
                    self.configuration_ready
                    and self.fcu_connected
                    and not critical
                    and self.is_data_fresh(
                        self.last_fcu_state_time,
                        self.fcu_state_timeout_sec,
                        now,
                    )
                ):
                    self.transition_to(
                        MissionState.WAIT_FCU, 'fault_cleared_recheck_required'
                    )

    def _target_in_configured_area(self, latitude, longitude):
        return (
            self.target_min_latitude <= latitude <= self.target_max_latitude
            and self.target_min_longitude <= longitude <= self.target_max_longitude
        )

    def _target_insert_window_blocker_locked(self, _now=None):
        if not self.reject_target_at_or_after_insert_wp:
            return None
        if self.current_mission_seq is None:
            return 'mission_current_seq_unknown'
        # WaypointList is event-driven: MAVROS republishes it when the mission or
        # current sequence changes, not as a periodic heartbeat.  Its age cannot
        # indicate whether the cached sequence is valid.  The cache is instead
        # invalidated on FCU disconnect, while normal FCU freshness gates and the
        # FCU interface's pre-upload pull protect the final mutation.
        if self.current_mission_count <= 0:
            return 'mission_waypoints_empty'
        if self.insert_wp_index >= self.current_mission_count:
            return (
                'insert_wp_index_out_of_mission:'
                f'insert_wp_index={self.insert_wp_index}:'
                f'mission_count={self.current_mission_count}'
            )
        if self.current_mission_seq >= self.insert_wp_index:
            return (
                'after_insert_wp_index:'
                f'current_seq={self.current_mission_seq}:'
                f'insert_wp_index={self.insert_wp_index}'
            )
        return None

    def target_callback(self, msg):
        now = time.monotonic()
        generation = None

        with self._lock:
            if self.attack_executed:
                self.get_logger().warning(
                    self._prefix(state='REJECTED')
                    + ' target rejected reason=attack_already_executed'
                )
                return

            if self.state != MissionState.STANDBY:
                self.get_logger().warning(
                    self._prefix(state='REJECTED')
                    + f' target rejected reason=invalid_state '
                    f'current_state={self.state.value}'
                )
                return

            if self.busy:
                self.get_logger().warning(
                    self._prefix(state='REJECTED')
                    + ' target rejected reason=busy'
                )
                return

            if not self.valid_lat_lon(msg.latitude, msg.longitude):
                self.get_logger().warning(
                    self._prefix(state='REJECTED')
                    + ' target rejected reason=invalid_coordinates'
                )
                return

            if not self._target_in_configured_area(
                msg.latitude,
                msg.longitude,
            ):
                self.get_logger().warning(
                    self._prefix(state='REJECTED')
                    + ' target rejected reason=outside_configured_bounds'
                )
                return

            if not self.valid_heading(msg.heading_deg):
                self.get_logger().warning(
                    self._prefix(state='REJECTED')
                    + ' target rejected reason=invalid_heading'
                )
                return

            insert_window_blocker = self._target_insert_window_blocker_locked(now)
            if insert_window_blocker:
                self.get_logger().warning(
                    self._prefix(state='REJECTED')
                    + ' target rejected reason=insert_window_closed '
                    + insert_window_blocker
                )
                return

            blockers = self._attack_start_blockers_locked(now)
            if blockers:
                self.get_logger().warning(
                    self._prefix(state='REJECTED')
                    + ' target rejected reason=attack_start_blocked '
                    + f'blockers={",".join(blockers)}'
                )
                return

            self.target_counter += 1

            self.active_target = {
                'id': f'target_{self.target_counter:03d}',
                'lat': float(msg.latitude),
                'lon': float(msg.longitude),
                'heading_deg': float(msg.heading_deg),
                'received_monotonic': now,
                'received_wall_time': time.time(),
                'session_id': self.boot_session_id,
                'received_after_boot': now >= self.boot_monotonic_time,
                'source_timestamp_available': False,
            }
            self.target_valid = True
            self._publish_target_valid_locked()

            self.get_logger().info(
                self._prefix()
                + ' target accepted auto_execute=true '
                f'lat={self.active_target["lat"]:.7f} '
                f'lon={self.active_target["lon"]:.7f} '
                f'heading_deg={self.active_target["heading_deg"]:.2f} '
                f'current_mission_seq={self.current_mission_seq} '
                f'insert_wp_index={self.insert_wp_index} '
                f'boot_session_id={self.boot_session_id}'
            )

            if not self.transition_to(
                MissionState.EXECUTING,
                'attack_start_gate_passed',
            ):
                self._invalidate_target_locked()
                return

            self.attack_executed = True
            generation = self.execution_generation

            self._publish_task_id_locked(
                self.active_target['id']
            )

        self.goto_target_process_point(generation)

    def disable_callback(self, request, response):
        del request

        with self._lock:
            if self.state == MissionState.EXECUTING:
                self.enter_safe(
                    'explicit_mission_disable_during_execution'
                )
            else:
                self.execution_allowed = False
                self._invalidate_target_locked()

            response.success = True
            response.message = 'Mission execution disabled.'
            return response

    def goto_target_process_point(self, generation):
        with self._lock:
            if (
                generation != self.execution_generation
                or self.state != MissionState.EXECUTING
                or not self.execution_allowed
                or self.active_target is None
            ):
                return
            target = dict(self.active_target)

        if not self.goto_client.service_is_ready():
            self.enter_safe('/fcu/goto_global service not available')
            return
        request = GoToGlobal.Request()
        request.latitude = target['lat']
        request.longitude = target['lon']
        request.acceptance_radius = self.acceptance_radius_m
        request.heading_deg = target['heading_deg']
        future = self.goto_client.call_async(request)
        future.add_done_callback(
            lambda completed: self.goto_done_callback(completed, generation)
        )
        self.get_logger().info(
            self._prefix(state='EXECUTING') + ' goto_global request sent'
        )

    @staticmethod
    def parse_key_value_text(text):
        values = {}
        for token in str(text).split():
            if '=' not in token:
                continue
            key, value = token.split('=', 1)
            values[key] = value
        return values

    def composite_mission_complete_callback(self, msg):
        fields = self.parse_key_value_text(msg.data)
        with self._lock:
            if self.state != MissionState.EXECUTING:
                self.get_logger().warning(
                    self._prefix('SAFETY')
                    + ' composite completion ignored reason=not_executing '
                    f'event="{msg.data}"'
                )
                return
            if self.active_target is None:
                self.get_logger().warning(
                    self._prefix('SAFETY')
                    + ' composite completion ignored reason=no_active_target '
                    f'event="{msg.data}"'
                )
                return
            event_task_id = fields.get('task_id')
            active_task_id = self.active_target['id']
            if event_task_id != active_task_id:
                self.get_logger().warning(
                    self._prefix('SAFETY')
                    + ' composite completion ignored reason=task_id_mismatch '
                    f'expected={active_task_id} actual={event_task_id} '
                    f'event="{msg.data}"'
                )
                return
            if fields.get('mission_type') != 'COMPOSITE':
                self.get_logger().warning(
                    self._prefix('SAFETY')
                    + ' composite completion ignored reason=mission_type_mismatch '
                    f'event="{msg.data}"'
                )
                return
            self.get_logger().info(
                self._prefix()
                + ' composite target segment completion received '
                f'd_seq={fields.get("final_seq", "unknown")} '
                f'c_confirmed={fields.get("c_confirmed", "unknown")} '
                f'c_min_distance_m={fields.get("c_min_distance_m", "unknown")}'
            )
        self.complete_mission('composite_d_reached')

    def goto_done_callback(self, future, generation):
        with self._lock:
            if generation != self.execution_generation or self.state != MissionState.EXECUTING:
                self.get_logger().warning(
                    self._prefix('SAFETY')
                    + ' stale goto result ignored reason=execution_generation_changed'
                )
                return
        try:
            result = future.result()
        except Exception as exc:
            self.enter_safe(f'goto_service_failed:{exc}')
            return
        if not result.success:
            self.enter_safe(f'goto_rejected:{result.message}')
            return
        self.get_logger().info(
            self._prefix() + ' goto_global request completed success=true'
        )
        self.get_logger().info(
            self._prefix()
            + ' composite mission uploaded; state remains EXECUTING '
            'waiting_for=d_reached'
        )

    def complete_mission(self, reason):
        with self._lock:
            if self.state != MissionState.EXECUTING:
                return
            self.transition_to(MissionState.MISSION_COMPLETE, reason)
            self.transition_to(MissionState.STANDBY, 'mission_authority_reset')

    def fail(self, reason):
        self.enter_safe(f'task_control_failure:{reason}')

    def _status_snapshot_locked(self):
        now = time.monotonic()
        attack_start_blockers = self._attack_start_blockers_locked(now)
        target_insert_window_blocker = self._target_insert_window_blocker_locked(now)
        ready_for_attack = (
            self.state == MissionState.STANDBY
            and not self.attack_executed
            and not self.busy
            and not attack_start_blockers
        )
        runtime_fault_durations = {
            fault: round(
                max(0.0, now - self.runtime_fault_since.get(fault, now)),
                3,
            )
            for fault in self.runtime_faults
        }
        return {
            'boot_session_id': self.boot_session_id,
            'state': self.state.value,
            'fcu_connected': self.fcu_connected,
            'armed': self.fcu_armed,
            'mode': self.fcu_mode,
            'expected_system_id': self.expected_system_id,
            'actual_system_id': self.actual_system_id,
            'expected_component_id': self.expected_component_id,
            'actual_component_id': self.actual_component_id,
            'heartbeat_count': self.heartbeat_count,
            'heartbeat_age_sec': self.data_age(self.last_heartbeat_time, now),
            'fcu_state_link_count': self.heartbeat_count,
            'fcu_state_link_age_sec': self.data_age(
                self.last_fcu_state_time, now
            ),
            'fcu_identity_confirmed': self._fcu_identity_confirmed_locked(),
            'fcu_state_link_stable': self._fcu_state_link_stable_locked(now),
            'fcu_state_age_sec': self.data_age(self.last_fcu_state_time, now),
            'current_mission_seq': self.current_mission_seq,
            'current_mission_count': self.current_mission_count,
            'mission_waypoints_age_sec': self.data_age(
                self.last_mission_waypoints_time,
                now,
            ),
            'insert_wp_index': self.insert_wp_index,
            'reject_target_at_or_after_insert_wp': (
                self.reject_target_at_or_after_insert_wp
            ),
            'target_insert_window_open': target_insert_window_blocker is None,
            'target_insert_window_blocker': target_insert_window_blocker,
            'position_age_sec': self.data_age(self.last_position_time, now),
            'gps_age_sec': self.data_age(self.last_gps_time, now),
            'ekf_age_sec': self.data_age(self.last_ekf_time, now),
            'ekf_source': self.ekf_source,
            'sensor_age_sec': self.data_age(self.last_sensor_time, now),
            'gps_healthy': self.gps_healthy,
            'gps_fix_type': self.gps_fix_type,
            'gps_satellites': self.gps_satellites,
            'gps_h_acc_m': (
                self.gps_h_acc_m if math.isfinite(self.gps_h_acc_m) else None
            ),
            'ekf_healthy': self.ekf_healthy,
            'required_sensors_healthy': self.sensor_health,
            'optional_sensor_warning': (
                self.optional_fcu_sensor_warning
                or any(
                    not self.is_data_fresh(
                        self.external_health_time.get(topic),
                        self.sensor_timeout_sec,
                        now,
                    )
                    or not self.external_health.get(topic, False)
                    for topic in self.optional_external_health_topics
                )
            ),
            'external_health': dict(self.external_health),
            'parameters_loaded': self.parameters_loaded,
            'parameters_valid': self.parameters_valid,
            'configuration_ready': self.configuration_ready,
            'execution_allowed': self.execution_allowed,
            'attack_executed': self.attack_executed,
            'target_valid': self.target_valid,
            'ready_for_attack': ready_for_attack,
            'attack_ready': ready_for_attack,
            'attack_start_blockers': attack_start_blockers,
            'runtime_health_level': self.runtime_health_level.value,
            'runtime_faults': list(self.runtime_faults),
            'runtime_fault_durations_sec': runtime_fault_durations,
            'runtime_fault_count': len(self.runtime_faults),
            'runtime_warning_count': (
                len(self.runtime_faults) - len(self.runtime_critical_faults)
            ),
            'runtime_critical_count': len(self.runtime_critical_faults),
            'safe_reason': self.safe_reason,
            'last_transition_reason': self.last_transition_reason,
        }

    def print_state(self):
        with self._lock:
            snapshot = self._status_snapshot_locked()
            standby_waiting_for = (
                'attack_already_executed'
                if self.attack_executed
                else (
                    'valid_target'
                    if snapshot['ready_for_attack']
                    else 'attack_prerequisites'
                )
            )
            waiting_for = {
                MissionState.WAIT_FCU: 'stable_fcu_identity_and_state_link',
                MissionState.STANDBY: standby_waiting_for,

                MissionState.EXECUTING: 'temporary_task',
                MissionState.SAFE: 'fault_recovery',
            }.get(self.state, 'state_transition')
            message = String()
            message.data = json.dumps(snapshot, separators=(',', ':'), sort_keys=True)
            self.diagnostic_publisher.publish(message)
            self.get_logger().info(
                self._prefix('STATUS')
                + f' busy={self._bool(self.busy)} waiting_for={waiting_for} '
                f'fcu_connected={self._bool(self.fcu_connected)} '
                f'armed={self._bool(self.fcu_armed)} mode={self.fcu_mode} '
                f'system_id={self.actual_system_id} '
                f'component_id={self.actual_component_id} '
                f'fcu_state_link_age_s={snapshot["fcu_state_link_age_sec"]} '
                f'current_mission_seq={snapshot["current_mission_seq"]} '
                f'insert_wp_index={snapshot["insert_wp_index"]} '
                f'target_insert_window_open='
                f'{self._bool(snapshot["target_insert_window_open"])} '
                f'execution_allowed={self._bool(self.execution_allowed)} '
                f'attack_executed={self._bool(self.attack_executed)} '
                f'target_valid={self._bool(self.target_valid)} '
                f'ready_for_attack={self._bool(snapshot["ready_for_attack"])} '
                f'runtime_health_level={snapshot["runtime_health_level"]} '
                f'runtime_fault_count={snapshot["runtime_fault_count"]} '
                f'safe_reason={self.safe_reason}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
