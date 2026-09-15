"""ROS 2 node that joins vision events with timestamped MAVROS telemetry."""

from collections import deque
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import time

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import GPSRAW
from mavros_msgs.srv import MessageInterval, StreamRate
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from uav_interfaces.msg import ReconTarget

from .core import (
    CameraModel,
    Observation,
    PixelFramePacketManager,
    TimedBuffer,
    TrackManager,
    geodetic_delta_m,
    hermite_tuple,
    image_center_weight,
    is_empty_target_label,
    lerp_tuple,
    project_pixel_to_ground,
    propagate_geodetic_with_local_delta,
    quat_slerp,
    resolve_packet_candidates,
)


def message_time(message):
    header = getattr(message, 'header', None)
    stamp = getattr(header, 'stamp', None)
    if stamp is not None and stamp.sec > 100000000:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return time.time()


def atomic_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', suffix='.tmp', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_copy(source: Path, destination: Path):
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + '.tmp')
    shutil.copyfile(source, temporary)
    os.replace(temporary, destination)


class ReconGeolocatorNode(Node):
    def __init__(self):
        super().__init__('recon_geolocator')
        self._declare_parameters()
        self.manifest_path = Path(self.get_parameter('manifest_path').value)
        self.frame_source_path = Path(self.get_parameter('frame_source_path').value)
        self.output_root = Path(self.get_parameter('output_root').value)
        self.output_root.mkdir(parents=True, exist_ok=True)
        self.status_path = self.output_root / 'status.json'

        self.positions = TimedBuffer(15.0)
        self.attitudes = TimedBuffer(15.0)
        self.local_pose_attitudes = TimedBuffer(15.0)
        self.local_positions = TimedBuffer(15.0)
        self.local_velocities = TimedBuffer(15.0)
        self.position_arrivals = deque(maxlen=120)
        self.attitude_arrivals = deque(maxlen=120)
        self.local_position_arrivals = deque(maxlen=240)
        self.local_velocity_arrivals = deque(maxlen=240)
        self.relative_altitudes = TimedBuffer(15.0)
        self.rtk_fixes = TimedBuffer(15.0)
        self.tracking_mode = str(self.get_parameter('tracking_mode').value)
        if self.tracking_mode not in {'pixel_packets', 'legacy_geo_cluster'}:
            raise ValueError(
                'tracking_mode must be pixel_packets or legacy_geo_cluster'
            )
        self.tracks = TrackManager(float(self.get_parameter('association_radius_m').value))
        self.frame_packets = PixelFramePacketManager(
            float(self.get_parameter('packet_gap_timeout_sec').value),
            float(self.get_parameter('packet_pixel_gate_base_px').value),
            float(self.get_parameter('packet_pixel_gate_rate_px_per_sec').value),
            float(self.get_parameter('packet_pixel_gate_max_px').value),
            int(self.get_parameter('packet_max_observations').value),
        )
        self.final_target_count = 0
        self.packet_snapshot_scores = {}
        self.packet_clock_capture_time = None
        self.packet_clock_monotonic = None
        self.last_manifest_key = None
        self.pending_manifests = deque()
        self.last_status = None
        self.last_status_write = 0.0
        self.last_result_write = {}
        self.last_snapshot_write = {}

        self.publisher = self.create_publisher(ReconTarget, '/vision/recon_result', 10)
        self.create_subscription(
            NavSatFix,
            self.get_parameter('global_position_topic').value,
            self._position_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').value,
            self._imu_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('local_pose_topic').value,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            self.get_parameter('local_velocity_topic').value,
            self._velocity_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64,
            self.get_parameter('relative_altitude_topic').value,
            self._relative_altitude_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            GPSRAW,
            self.get_parameter('gps_raw_topic').value,
            self._gps_raw_callback,
            qos_profile_sensor_data,
        )
        self.message_interval_client = self.create_client(
            MessageInterval, '/mavros/set_message_interval'
        )
        self.stream_rate_client = self.create_client(StreamRate, '/mavros/set_stream_rate')
        self.global_position_rate_future = None
        self.local_position_rate_future = None
        self.attitude_stream_rate_future = None
        self.global_position_rate_configured = False
        self.local_position_rate_configured = False
        self.attitude_stream_rate_configured = False
        self.last_global_position_rate_request = 0.0
        self.last_local_position_rate_request = 0.0
        self.last_attitude_stream_rate_request = 0.0
        self.create_timer(1.0, self._configure_mavlink_rates)
        poll_period = 1.0 / max(1.0, float(self.get_parameter('manifest_poll_hz').value))
        self.create_timer(poll_period, self._poll_manifest)
        self.get_logger().info(
            f'recon ready: manifest={self.manifest_path}, output={self.output_root}, '
            f'camera tilt={self.get_parameter("camera_forward_tilt_deg").value:.1f} deg forward, '
            f'{self.get_parameter("camera_left_tilt_deg").value:.1f} deg left, '
            f'position={self.get_parameter("position_solution_mode").value}, '
            f'tracking={self.tracking_mode}'
        )

    def _declare_parameters(self):
        values = {
            'manifest_path': '/home/nx163/youth-vision-runtime/overlays/latest_crops/manifest_fast.json',
            'frame_source_path': '/home/nx163/youth-vision-runtime/overlays/latest.jpg',
            'output_root': '/home/nx163/youth-vision-runtime/recon_results/current',
            'global_position_topic': '/mavros/global_position/global',
            'imu_topic': '/mavros/imu/data',
            'local_pose_topic': '/mavros/local_position/pose',
            'local_velocity_topic': '/mavros/local_position/velocity_local',
            'relative_altitude_topic': '/mavros/global_position/rel_alt',
            'gps_raw_topic': '/mavros/gpsstatus/gps1/raw',
            'manifest_poll_hz': 40.0,
            'configure_mavlink_rates': True,
            'global_position_rate_hz': 10.0,
            'local_position_rate_hz': 60.0,
            'attitude_stream_rate_hz': 60,
            'mavlink_rate_retry_sec': 30.0,
            'position_solution_mode': 'rtk_anchor_local_delta',
            'global_anchor_max_age_sec': 0.35,
            'local_position_max_gap_sec': 0.12,
            'local_velocity_max_gap_sec': 0.12,
            'local_delta_max_speed_mps': 80.0,
            'local_global_disagreement_max_m': 5.0,
            'telemetry_max_gap_sec': 0.35,
            'rtk_max_gap_sec': 0.35,
            'max_pending_manifests': 120,
            'rtk_fixed_min_fix_type': 6,
            'calibration_valid': False,
            'calibration_width': 1440,
            'calibration_height': 1080,
            'fx': 1738.23,
            'fy': 1737.08,
            'cx': 719.5,
            'cy': 539.5,
            'distortion': [0.0, 0.0, 0.0, 0.0, 0.0],
            'camera_forward_tilt_deg': 20.0,
            'camera_left_tilt_deg': 0.0,
            'camera_offset_flu_m': [0.0, 0.0, 0.0],
            'ground_altitude_mode': 'home_relative',
            'fixed_ground_altitude_msl_m': 0.0,
            'fixed_relative_altitude_m': 0.0,
            'tracking_mode': 'pixel_packets',
            'packet_gap_timeout_sec': 0.20,
            'packet_pixel_gate_base_px': 25.0,
            'packet_pixel_gate_rate_px_per_sec': 1300.0,
            'packet_pixel_gate_max_px': 200.0,
            'packet_max_observations': 300,
            'packet_min_observations': 11,
            'packet_merge_distance_m': 1.5,
            'packet_distinct_distance_m': 10.0,
            'association_radius_m': 3.0,
            'minimum_observations': 5,
            'minimum_observation_span_sec': 0.20,
            'minimum_label_consensus': 0.80,
            'single_observation_sigma_m': 0.15,
            'max_horizontal_radius_95_m': 0.50,
            'center_weight_minimum': 0.25,
            'center_weight_power': 1.0,
            'result_write_hz': 20.0,
            'snapshot_write_hz': 5.0,
        }
        for name, value in values.items():
            self.declare_parameter(name, value)

    def _position_callback(self, message):
        if message.status.status < 0:
            return
        self.position_arrivals.append(time.monotonic())
        self.positions.add(message_time(message), (message.latitude, message.longitude, message.altitude))

    def _pose_callback(self, message):
        timestamp = message_time(message)
        p = message.pose.position
        q = message.pose.orientation
        self.local_position_arrivals.append(time.monotonic())
        self.local_positions.add(timestamp, (p.x, p.y, p.z))
        self.local_pose_attitudes.add(timestamp, (q.x, q.y, q.z, q.w))

    def _velocity_callback(self, message):
        timestamp = message_time(message)
        velocity = message.twist.linear
        self.local_velocity_arrivals.append(time.monotonic())
        self.local_velocities.add(timestamp, (velocity.x, velocity.y, velocity.z))

    def _imu_callback(self, message):
        q = message.orientation
        self.attitude_arrivals.append(time.monotonic())
        self.attitudes.add(message_time(message), (q.x, q.y, q.z, q.w))

    def _relative_altitude_callback(self, message):
        self.relative_altitudes.add(time.time(), float(message.data))

    def _gps_raw_callback(self, message):
        h_acc_m = float(message.h_acc) * 0.001
        if message.h_acc == 0 or message.h_acc == 4294967295:
            h_acc_m = 0.0
        self.rtk_fixes.add(message_time(message), (int(message.fix_type), h_acc_m))

    def _configure_mavlink_rates(self):
        if not bool(self.get_parameter('configure_mavlink_rates').value):
            return
        now = time.monotonic()
        retry_sec = float(self.get_parameter('mavlink_rate_retry_sec').value)
        position_target = float(self.get_parameter('global_position_rate_hz').value)
        local_position_target = float(self.get_parameter('local_position_rate_hz').value)
        attitude_target = float(self.get_parameter('attitude_stream_rate_hz').value)
        position_rate_low = self._observed_rate_hz(self.position_arrivals) < position_target * 0.8
        local_position_rate_low = (
            self._observed_rate_hz(self.local_position_arrivals) < local_position_target * 0.8
        )
        attitude_rate_low = self._observed_rate_hz(self.attitude_arrivals) < attitude_target * 0.8
        if (
            position_rate_low
            and self.global_position_rate_future is None
            and self.message_interval_client.service_is_ready()
            and now - self.last_global_position_rate_request >= retry_sec
        ):
            request = MessageInterval.Request()
            request.message_id = 33  # MAVLink GLOBAL_POSITION_INT, FCU EKF global position.
            request.message_rate = position_target
            self.last_global_position_rate_request = now
            self.global_position_rate_future = self.message_interval_client.call_async(request)
            self.global_position_rate_future.add_done_callback(self._global_position_rate_response)
        if (
            local_position_rate_low
            and self.local_position_rate_future is None
            and self.message_interval_client.service_is_ready()
            and now - self.last_local_position_rate_request >= retry_sec
        ):
            request = MessageInterval.Request()
            request.message_id = 32  # MAVLink LOCAL_POSITION_NED, converted to ROS ENU by MAVROS.
            request.message_rate = local_position_target
            self.last_local_position_rate_request = now
            self.local_position_rate_future = self.message_interval_client.call_async(request)
            self.local_position_rate_future.add_done_callback(self._local_position_rate_response)
        if (
            attitude_rate_low
            and self.attitude_stream_rate_future is None
            and self.stream_rate_client.service_is_ready()
            and now - self.last_attitude_stream_rate_request >= retry_sec
        ):
            request = StreamRate.Request()
            request.stream_id = StreamRate.Request.STREAM_EXTRA1
            request.message_rate = int(attitude_target)
            request.on_off = True
            self.last_attitude_stream_rate_request = now
            self.attitude_stream_rate_future = self.stream_rate_client.call_async(request)
            self.attitude_stream_rate_future.add_done_callback(self._attitude_stream_rate_response)

    @staticmethod
    def _observed_rate_hz(arrivals):
        if len(arrivals) < 5:
            return 0.0
        duration = arrivals[-1] - arrivals[0]
        return 0.0 if duration <= 0.0 else (len(arrivals) - 1) / duration

    def _global_position_rate_response(self, future):
        try:
            response = future.result()
            self.global_position_rate_configured = bool(response.success)
            if self.global_position_rate_configured:
                self.get_logger().info(
                    f'FCU global position rate requested at '
                    f'{self.get_parameter("global_position_rate_hz").value:.1f} Hz'
                )
            else:
                self.get_logger().warn('FCU rejected the GLOBAL_POSITION_INT rate request')
        except Exception as error:  # noqa: BLE001 - ROS future reports transport errors here.
            self.get_logger().warn(f'global position rate request failed: {error}')
        finally:
            self.global_position_rate_future = None

    def _local_position_rate_response(self, future):
        try:
            response = future.result()
            self.local_position_rate_configured = bool(response.success)
            if self.local_position_rate_configured:
                self.get_logger().info(
                    f'FCU local EKF position rate requested at '
                    f'{self.get_parameter("local_position_rate_hz").value:.1f} Hz'
                )
            else:
                self.get_logger().warn('FCU rejected the LOCAL_POSITION_NED rate request')
        except Exception as error:  # noqa: BLE001 - ROS future reports transport errors here.
            self.get_logger().warn(f'local position rate request failed: {error}')
        finally:
            self.local_position_rate_future = None

    def _attitude_stream_rate_response(self, future):
        try:
            response = future.result()
            self.attitude_stream_rate_configured = bool(response.success)
            if self.attitude_stream_rate_configured:
                self.get_logger().info(
                    f'FCU attitude stream requested at '
                    f'{self.get_parameter("attitude_stream_rate_hz").value} Hz'
                )
            else:
                self.get_logger().warn('FCU rejected the attitude stream rate request')
        except Exception as error:  # noqa: BLE001 - ROS future reports transport errors here.
            self.get_logger().warn(f'attitude stream rate request failed: {error}')
        finally:
            self.attitude_stream_rate_future = None

    def _set_status(self, status, detail=None):
        now = time.time()
        if status == self.last_status and now - self.last_status_write < 1.0:
            return
        record = {
            'timestamp_unix_s': now,
            'status': status,
            'detail': detail or '',
            'rtk_required': True,
            'coordinate_valid': False,
            'telemetry_rates_hz': {
                'rtk_global': round(self._observed_rate_hz(self.position_arrivals), 1),
                'local_position': round(self._observed_rate_hz(self.local_position_arrivals), 1),
                'local_velocity': round(self._observed_rate_hz(self.local_velocity_arrivals), 1),
                'attitude': round(self._observed_rate_hz(self.attitude_arrivals), 1),
            },
        }
        atomic_json(self.status_path, record)
        if status != self.last_status:
            self.get_logger().warn(f'{status}: {detail or ""}')
        self.last_status = status
        self.last_status_write = now

    def _poll_manifest(self):
        try:
            with self.manifest_path.open('r', encoding='utf-8') as stream:
                manifest = json.load(stream)
        except (OSError, json.JSONDecodeError):
            self._set_status('waiting_for_vision', str(self.manifest_path))
            return
        capture_ns = int(manifest.get('capture_timestamp_unix_ns', 0))
        frame_number = int(manifest.get('frame', -1))
        key = (capture_ns, frame_number)
        if key != self.last_manifest_key:
            self.last_manifest_key = key
            if capture_ns <= 0:
                self._set_status(
                    'vision_timestamp_missing',
                    'runtime manifest must provide capture_timestamp_unix_ns',
                )
            else:
                self.pending_manifests.append(manifest)
                maximum = max(1, int(self.get_parameter('max_pending_manifests').value))
                while len(self.pending_manifests) > maximum:
                    self.pending_manifests.popleft()
                    self._set_status(
                        'telemetry_queue_overflow',
                        f'pending vision manifests exceeded {maximum}',
                    )
        self._drain_pending_manifests()
        if self.tracking_mode == 'pixel_packets' and not self.pending_manifests:
            packet_time = self._packet_time_now()
            if packet_time is not None:
                self._finalize_packet_groups(
                    self.frame_packets.advance(packet_time)
                )

    def _drain_pending_manifests(self):
        while self.pending_manifests:
            manifest = self.pending_manifests[0]
            if not self._process_manifest(manifest):
                return
            if self.tracking_mode == 'pixel_packets':
                capture_ns = int(manifest.get('capture_timestamp_unix_ns', 0))
                if capture_ns > 0:
                    self.packet_clock_capture_time = capture_ns * 1e-9
                    self.packet_clock_monotonic = time.monotonic()
            self.pending_manifests.popleft()

    def _packet_time_now(self):
        if (
            self.packet_clock_capture_time is None
            or self.packet_clock_monotonic is None
        ):
            return None
        return self.packet_clock_capture_time + max(
            0.0,
            time.monotonic() - self.packet_clock_monotonic,
        )

    def _local_position_at(self, timestamp):
        max_gap = float(self.get_parameter('local_position_max_gap_sec').value)
        before, after = self.local_positions.bracket(timestamp)
        if before is None or after is None:
            return None, 'unavailable'
        if timestamp - before.timestamp > max_gap or after.timestamp - timestamp > max_gap:
            return None, 'gap_exceeded'
        if before.timestamp == after.timestamp:
            return before.value, 'sample'

        ratio = (timestamp - before.timestamp) / (after.timestamp - before.timestamp)
        velocity_gap = float(self.get_parameter('local_velocity_max_gap_sec').value)
        velocity0 = self.local_velocities.nearest(before.timestamp, velocity_gap)
        velocity1 = self.local_velocities.nearest(after.timestamp, velocity_gap)
        if velocity0 is not None and velocity1 is not None:
            return (
                hermite_tuple(
                    before.value,
                    velocity0,
                    after.value,
                    velocity1,
                    ratio,
                    after.timestamp - before.timestamp,
                ),
                'cubic_hermite',
            )
        return lerp_tuple(before.value, after.value, ratio), 'linear'

    def _resolve_aircraft_position(self, timestamp, max_gap):
        global_interpolated = self.positions.interpolate_bracketed(timestamp, lerp_tuple, max_gap)
        fallback = {
            'position_source': 'global_interpolation',
            'global_anchor_age_sec': 0.0,
            'global_anchor_timestamp_unix_s': timestamp,
            'global_anchor_lla': global_interpolated,
            'local_position_method': 'none',
            'local_delta_enu_m': (0.0, 0.0, 0.0),
            'global_local_disagreement_m': 0.0,
        }
        if str(self.get_parameter('position_solution_mode').value) != 'rtk_anchor_local_delta':
            waiting = global_interpolated is None and not self.positions.has_sample_at_or_after(
                timestamp
            )
            return global_interpolated, fallback, waiting

        anchor_max_age = float(self.get_parameter('global_anchor_max_age_sec').value)
        anchor = self.positions.latest_at_or_before(timestamp, anchor_max_age)
        if anchor is None:
            waiting = global_interpolated is None and not self.positions.has_sample_at_or_after(
                timestamp
            )
            return global_interpolated, fallback, waiting

        frame_local, frame_method = self._local_position_at(timestamp)
        anchor_local, anchor_method = self._local_position_at(anchor.timestamp)
        if frame_local is None or anchor_local is None:
            waiting = global_interpolated is None and not (
                self.local_positions.has_sample_at_or_after(timestamp)
                and self.positions.has_sample_at_or_after(timestamp)
            )
            return global_interpolated, fallback, waiting

        delta = tuple(frame - origin for frame, origin in zip(frame_local, anchor_local))
        age = max(0.0, timestamp - anchor.timestamp)
        distance = math.sqrt(sum(component * component for component in delta))
        speed_limit = float(self.get_parameter('local_delta_max_speed_mps').value)
        allowed_distance = max(1.0, speed_limit * max(age, 0.02))
        if distance > allowed_distance:
            self.get_logger().warn(
                f'local EKF displacement rejected: {distance:.2f} m in {age:.3f} s '
                f'(limit {allowed_distance:.2f} m)'
            )
            waiting = global_interpolated is None and not self.positions.has_sample_at_or_after(
                timestamp
            )
            return global_interpolated, fallback, waiting

        propagated = propagate_geodetic_with_local_delta(anchor.value, anchor_local, frame_local)
        disagreement = 0.0
        if global_interpolated is not None:
            east, north = geodetic_delta_m(
                global_interpolated[0],
                global_interpolated[1],
                propagated[0],
                propagated[1],
            )
            disagreement = math.hypot(east, north)
            disagreement_limit = float(
                self.get_parameter('local_global_disagreement_max_m').value
            )
            if disagreement > disagreement_limit:
                self.get_logger().warn(
                    f'local/global position disagreement {disagreement:.2f} m exceeds '
                    f'{disagreement_limit:.2f} m; using global interpolation'
                )
                return global_interpolated, fallback, False

        context = {
            'position_source': 'rtk_anchor_local_delta',
            'global_anchor_age_sec': age,
            'global_anchor_timestamp_unix_s': anchor.timestamp,
            'global_anchor_lla': anchor.value,
            'local_position_method': f'{anchor_method}+{frame_method}',
            'local_delta_enu_m': delta,
            'global_local_disagreement_m': disagreement,
        }
        return propagated, context, False

    def _process_manifest(self, manifest):
        capture_ns = int(manifest['capture_timestamp_unix_ns'])
        capture_time = capture_ns * 1e-9
        max_gap = float(self.get_parameter('telemetry_max_gap_sec').value)
        rtk_max_gap = float(self.get_parameter('rtk_max_gap_sec').value)

        has_attitude_after = (
            self.attitudes.has_sample_at_or_after(capture_time)
            or self.local_pose_attitudes.has_sample_at_or_after(capture_time)
        )
        if not has_attitude_after:
            self._set_status(
                'waiting_for_fcu_telemetry',
                f'waiting for attitude after camera timestamp; '
                f'pending={len(self.pending_manifests)}',
            )
            return False

        position, position_context, position_pending = self._resolve_aircraft_position(
            capture_time, max_gap
        )
        if position_pending:
            self._set_status(
                'waiting_for_fcu_telemetry',
                f'waiting for frame-time position; pending={len(self.pending_manifests)}',
            )
            return False
        attitude = self.attitudes.interpolate_bracketed(capture_time, quat_slerp, max_gap)
        if attitude is None:
            attitude = self.local_pose_attitudes.interpolate_bracketed(
                capture_time, quat_slerp, max_gap
            )
        fix_sample = self.rtk_fixes.latest_at_or_before(capture_time, rtk_max_gap)
        if fix_sample is None and not self.rtk_fixes.has_sample_at_or_after(capture_time):
            self._set_status(
                'waiting_for_fcu_telemetry',
                f'waiting for RTK status; pending={len(self.pending_manifests)}',
            )
            return False
        fix = fix_sample.value if fix_sample is not None else self.rtk_fixes.nearest(
            capture_time, rtk_max_gap
        )
        if position is None or attitude is None:
            self._set_status(
                'telemetry_alignment_rejected',
                f'camera frame lacks bracketing position/attitude within {max_gap:.3f} s',
            )
            return True
        if fix is None:
            self._set_status(
                'telemetry_alignment_rejected',
                f'camera frame lacks RTK status within {rtk_max_gap:.3f} s',
            )
            return True
        if fix[0] < int(self.get_parameter('rtk_fixed_min_fix_type').value):
            self._set_status('waiting_for_rtk_fixed', f'GPS fix_type={fix[0]}')
            return True
        if not bool(self.get_parameter('calibration_valid').value):
            self._set_status('waiting_for_camera_calibration', 'calibration_valid is false')
            return True
        mode = str(self.get_parameter('ground_altitude_mode').value)
        ground_altitude = self._ground_altitude(
            capture_time, position[2], max_gap, position_context
        )
        if ground_altitude is None:
            if mode == 'home_relative' and not self.relative_altitudes.has_sample_at_or_after(
                capture_time
            ):
                self._set_status(
                    'waiting_for_fcu_telemetry',
                    f'waiting for relative altitude; pending={len(self.pending_manifests)}',
                )
                return False
            self._set_status(
                'telemetry_alignment_rejected',
                f'no bracketing ground-altitude sample for mode={mode}',
            )
            return True
        frame_width = int(manifest.get('frame_width', 0))
        frame_height = int(manifest.get('frame_height', 0))
        if frame_width <= 0 or frame_height <= 0:
            self._set_status('vision_geometry_missing', 'frame dimensions absent from manifest')
            return True
        camera = self._camera_model(frame_width, frame_height)
        crop_root = self.manifest_path.parent
        if self.tracking_mode == 'pixel_packets':
            self._finalize_packet_groups(self.frame_packets.advance(capture_time))
        projected = 0
        frame_number = int(manifest.get('frame', -1))
        source_sequence = int(manifest.get('source_sequence', 0))
        for crop in manifest.get('crops', []):
            label = str(crop.get('class_label', ''))
            if is_empty_target_label(label):
                continue
            center = crop.get('center')
            if not center or len(center) != 2:
                continue
            crop_path = crop_root / crop.get('src', '')
            try:
                coordinate = project_pixel_to_ground(
                    center, position, attitude, ground_altitude, camera
                )
            except ValueError as error:
                self._set_status('projection_rejected', str(error))
                continue
            observation = Observation(
                timestamp=capture_time,
                latitude=coordinate.latitude,
                longitude=coordinate.longitude,
                altitude_msl_m=coordinate.altitude_msl_m,
                label=label,
                class_id=int(crop.get('class_id', -1)),
                confidence=float(crop.get('class_prob', 0.0)),
                pose_score=float(crop.get('pose_score', 0.0)),
                frame_path=str(self.frame_source_path),
                crop_path=str(crop_path),
                horizontal_sigma_m=fix[1],
                center_distance_norm=min(
                    1.0,
                    math.hypot(
                        (float(center[0]) - camera.cx) / (frame_width * 0.5),
                        (float(center[1]) - camera.cy) / (frame_height * 0.5),
                    ),
                ),
                position_source=position_context['position_source'],
                global_anchor_age_sec=position_context['global_anchor_age_sec'],
                local_position_method=position_context['local_position_method'],
                local_delta_enu_m=position_context['local_delta_enu_m'],
                frame_number=frame_number,
                source_sequence=source_sequence,
                center_px=(float(center[0]), float(center[1])),
            )
            if self.tracking_mode == 'pixel_packets':
                packet = self.frame_packets.add(observation)
                self._preserve_packet_snapshot(packet.packet_id, observation)
            else:
                track = self.tracks.add(observation)
                self._write_track(track)
            projected += 1
        if self.tracking_mode == 'pixel_packets':
            self._finalize_packet_groups(
                self.frame_packets.take_limit_reached_group()
            )
        if projected:
            tracker_detail = (
                f'active_packets={self.frame_packets.active_packet_count}, '
                f'pending_packets={self.frame_packets.pending_packet_count}'
                if self.tracking_mode == 'pixel_packets'
                else f'tracks={len(self.tracks.tracks)}'
            )
            self._set_status(
                'tracking',
                f'projected={projected}, {tracker_detail}, '
                f'position={position_context["position_source"]}',
            )
        else:
            self._set_status('waiting_for_target', 'manifest contains no usable target crops')
        return True

    def _ground_altitude(self, timestamp, aircraft_altitude, max_gap, position_context):
        mode = str(self.get_parameter('ground_altitude_mode').value)
        if mode == 'fixed_msl':
            return float(self.get_parameter('fixed_ground_altitude_msl_m').value)
        if mode == 'fixed_relative':
            relative = float(self.get_parameter('fixed_relative_altitude_m').value)
            if not math.isfinite(relative) or relative <= 0.0:
                return None
            return aircraft_altitude - relative
        if mode == 'home_relative':
            if position_context['position_source'] == 'rtk_anchor_local_delta':
                anchor_time = position_context['global_anchor_timestamp_unix_s']
                anchor_lla = position_context['global_anchor_lla']
                relative = self.relative_altitudes.nearest(
                    anchor_time,
                    float(self.get_parameter('global_anchor_max_age_sec').value),
                )
                return None if relative is None else anchor_lla[2] - relative
            relative = self.relative_altitudes.interpolate(
                timestamp, lambda a, b, r: a + r * (b - a), max_gap
            )
            return None if relative is None else aircraft_altitude - relative
        return None

    def _camera_model(self, frame_width, frame_height):
        width_scale = frame_width / float(self.get_parameter('calibration_width').value)
        height_scale = frame_height / float(self.get_parameter('calibration_height').value)
        return CameraModel(
            fx=float(self.get_parameter('fx').value) * width_scale,
            fy=float(self.get_parameter('fy').value) * height_scale,
            cx=float(self.get_parameter('cx').value) * width_scale,
            cy=float(self.get_parameter('cy').value) * height_scale,
            distortion=list(self.get_parameter('distortion').value),
            forward_tilt_deg=float(self.get_parameter('camera_forward_tilt_deg').value),
            offset_flu_m=list(self.get_parameter('camera_offset_flu_m').value),
            left_tilt_deg=float(self.get_parameter('camera_left_tilt_deg').value),
        )

    def _preserve_packet_snapshot(self, packet_id, observation):
        score = (
            observation.confidence
            * observation.pose_score
            * image_center_weight(
                observation.center_distance_norm,
                float(self.get_parameter('center_weight_minimum').value),
                float(self.get_parameter('center_weight_power').value),
            )
        )
        if score <= self.packet_snapshot_scores.get(packet_id, -1.0):
            return
        source_frame = Path(observation.frame_path)
        source_crop = Path(observation.crop_path)
        if not source_frame.is_file() or not source_crop.is_file():
            return
        cache_dir = self.output_root / '.frame_packets' / packet_id
        frame_destination = cache_dir / 'frame.jpg'
        crop_destination = cache_dir / 'crop_128.jpg'
        try:
            atomic_copy(source_frame, frame_destination)
            atomic_copy(source_crop, crop_destination)
        except OSError as error:
            self.get_logger().warn(
                f'packet snapshot failed packet_id={packet_id}: {error}'
            )
            return
        observation.frame_path = str(frame_destination)
        observation.crop_path = str(crop_destination)
        self.packet_snapshot_scores[packet_id] = score

    def _finalize_packet_groups(self, groups):
        for group in groups:
            minimum = max(
                1,
                int(self.get_parameter('packet_min_observations').value),
            )
            candidates = []
            rejected = []
            for packet in group.packets:
                raw_count = len(packet.observations)
                if raw_count < minimum:
                    rejected.append({
                        'packet_id': packet.packet_id,
                        'reason': 'insufficient_frames',
                        'frame_count': raw_count,
                    })
                    continue
                fused = packet.fuse(
                    float(self.get_parameter('single_observation_sigma_m').value),
                    float(self.get_parameter('center_weight_minimum').value),
                    float(self.get_parameter('center_weight_power').value),
                )
                if fused['observation_count'] < minimum:
                    rejected.append({
                        'packet_id': packet.packet_id,
                        'reason': 'insufficient_frames_after_outlier_rejection',
                        'frame_count': fused['observation_count'],
                        'raw_frame_count': raw_count,
                    })
                    continue
                candidates.append(fused)

            winners, decisions = resolve_packet_candidates(
                candidates,
                float(self.get_parameter('packet_merge_distance_m').value),
                float(self.get_parameter('packet_distinct_distance_m').value),
            )
            self.get_logger().info(
                '[RECON_PACKET] stage=group_finalized '
                f'label={group.label} closure={group.closure_reason} '
                f'packets={len(group.packets)} '
                f'candidates={len(candidates)} winners={len(winners)} '
                f'rejected={json.dumps(rejected, ensure_ascii=False)} '
                f'decisions={json.dumps(decisions, ensure_ascii=False)}'
            )
            for fused in winners:
                self.final_target_count += 1
                target_id = f'target_{self.final_target_count:03d}'
                packet_metadata = {
                    'packet_ids': list(fused.get('packet_ids', [])),
                    'packet_count': int(fused.get('packet_count', 1)),
                    'raw_observation_count': int(
                        fused.get('raw_observation_count', fused['observation_count'])
                    ),
                    'label_counts': dict(fused.get('label_counts', {})),
                    'label_vote_method': 'valid_frame_count_majority',
                    'rejected_observation_count': int(
                        fused.get('rejected_observation_count', 0)
                    ),
                    'gap_timeout_sec': float(
                        self.get_parameter('packet_gap_timeout_sec').value
                    ),
                    'pixel_gate_max_px': float(
                        self.get_parameter('packet_pixel_gate_max_px').value
                    ),
                    'max_observations': int(
                        self.get_parameter('packet_max_observations').value
                    ),
                    'closure_reason': group.closure_reason,
                    'merge_distance_m': float(
                        self.get_parameter('packet_merge_distance_m').value
                    ),
                    'distinct_distance_m': float(
                        self.get_parameter('packet_distinct_distance_m').value
                    ),
                    'group_decisions': decisions,
                }
                self._emit_fused_result(
                    target_id,
                    fused,
                    valid=True,
                    status='finalized',
                    fusion_method='pixel_packet_robust_frame_weighted',
                    packet_metadata=packet_metadata,
                    throttle=False,
                )
                self.get_logger().info(
                    '[RECON_PACKET] stage=coordinate_finalized '
                    f'target_id={target_id} label={fused["label"]} '
                    f'packets={fused.get("packet_count", 1)} '
                    f'frames={fused["observation_count"]} '
                    f'lat={fused["latitude"]:.8f} '
                    f'lon={fused["longitude"]:.8f}'
                )

            for packet in group.packets:
                self.packet_snapshot_scores.pop(packet.packet_id, None)
                shutil.rmtree(
                    self.output_root / '.frame_packets' / packet.packet_id,
                    ignore_errors=True,
                )

    def _write_track(self, track):
        fused = track.fuse(
            float(self.get_parameter('single_observation_sigma_m').value),
            float(self.get_parameter('center_weight_minimum').value),
            float(self.get_parameter('center_weight_power').value),
        )
        valid = (
            fused['observation_count'] >= int(self.get_parameter('minimum_observations').value)
            and fused['observation_span_sec'] >= float(self.get_parameter('minimum_observation_span_sec').value)
            and fused['label_consensus'] >= float(self.get_parameter('minimum_label_consensus').value)
            and fused['horizontal_radius_95_m'] <= float(self.get_parameter('max_horizontal_radius_95_m').value)
        )
        status = 'confirmed' if valid else 'collecting_observations'
        self._emit_fused_result(
            track.target_id,
            fused,
            valid=valid,
            status=status,
            fusion_method='legacy_geo_cluster_weighted_mean',
            throttle=True,
        )

    def _emit_fused_result(
        self,
        target_id,
        fused,
        valid,
        status,
        fusion_method,
        packet_metadata=None,
        throttle=False,
    ):
        target_dir = self.output_root / target_id
        frame_destination = target_dir / 'frame.jpg'
        crop_destination = target_dir / 'crop_128.jpg'
        best = fused['best']
        record = {
            'target_id': target_id,
            'frame_path': f'{target_id}/frame.jpg',
            'crop_path': f'{target_id}/crop_128.jpg',
            'recognition': {
                'label': fused['label'],
                'class_id': fused['class_id'],
                'confidence': round(fused['confidence'], 6),
                'label_consensus': round(fused['label_consensus'], 6),
            },
            'coordinate': {
                'latitude': round(fused['latitude'], 8),
                'longitude': round(fused['longitude'], 8),
                'altitude_msl_m': round(fused['altitude_msl_m'], 3),
                'horizontal_radius_95_m': round(fused['horizontal_radius_95_m'], 3),
            },
            'observation_count': fused['observation_count'],
            'raw_observation_count': int(
                fused.get('raw_observation_count', fused['observation_count'])
            ),
            'rejected_observation_count': int(
                fused.get('rejected_observation_count', 0)
            ),
            'observation_span_sec': round(fused['observation_span_sec'], 3),
            'coordinate_fusion': {
                'method': fusion_method,
                'mean_center_weight': round(fused['mean_center_weight'], 6),
            },
            'telemetry_alignment': {
                'position_source': best.position_source,
                'global_anchor_age_sec': round(best.global_anchor_age_sec, 6),
                'local_position_method': best.local_position_method,
                'local_delta_enu_m': [round(float(value), 4) for value in best.local_delta_enu_m],
                'observed_rates_hz': {
                    'rtk_global': round(self._observed_rate_hz(self.position_arrivals), 1),
                    'local_position': round(
                        self._observed_rate_hz(self.local_position_arrivals), 1
                    ),
                    'local_velocity': round(
                        self._observed_rate_hz(self.local_velocity_arrivals), 1
                    ),
                    'attitude': round(self._observed_rate_hz(self.attitude_arrivals), 1),
                },
            },
            'rtk_fixed': True,
            'valid': valid,
            'status': status,
        }
        if packet_metadata is not None:
            record['frame_packet_fusion'] = packet_metadata
        message = ReconTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'wgs84'
        message.target_id = target_id
        message.frame_path = record['frame_path']
        message.crop_path = record['crop_path']
        message.label = fused['label']
        message.class_id = fused['class_id']
        message.confidence = fused['confidence']
        message.latitude = fused['latitude']
        message.longitude = fused['longitude']
        message.altitude_msl_m = fused['altitude_msl_m']
        message.horizontal_radius_95_m = fused['horizontal_radius_95_m']
        message.observation_count = fused['observation_count']
        message.rtk_fixed = True
        message.valid = valid
        message.status = status
        self.publisher.publish(message)

        now = time.monotonic()
        result_period = 1.0 / max(1.0, float(self.get_parameter('result_write_hz').value))
        if (
            not throttle
            or now - self.last_result_write.get(target_id, 0.0) >= result_period
        ):
            atomic_json(target_dir / 'result.json', record)
            self.last_result_write[target_id] = now

        snapshot_period = 1.0 / max(1.0, float(self.get_parameter('snapshot_write_hz').value))
        if (
            not throttle
            or now - self.last_snapshot_write.get(target_id, 0.0) >= snapshot_period
        ):
            source_frame = Path(best.frame_path)
            source_crop = Path(best.crop_path)
            if source_frame.is_file() and source_crop.is_file():
                atomic_copy(source_frame, frame_destination)
                atomic_copy(source_crop, crop_destination)
                self.last_snapshot_write[target_id] = now


def main(args=None):
    rclpy.init(args=args)
    node = ReconGeolocatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass
