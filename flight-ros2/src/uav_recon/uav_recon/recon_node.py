"""ROS 2 node that joins vision events with timestamped MAVROS telemetry."""

import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GPSRAW
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from uav_interfaces.msg import ReconTarget

from .core import (
    CameraModel,
    Observation,
    TimedBuffer,
    TrackManager,
    lerp_tuple,
    project_pixel_to_ground,
    quat_slerp,
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
        self.relative_altitudes = TimedBuffer(15.0)
        self.rtk_fixes = TimedBuffer(15.0)
        self.tracks = TrackManager(float(self.get_parameter('association_radius_m').value))
        self.last_manifest_key = None
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
            PoseStamped,
            self.get_parameter('local_pose_topic').value,
            self._pose_callback,
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
        poll_period = 1.0 / max(1.0, float(self.get_parameter('manifest_poll_hz').value))
        self.create_timer(poll_period, self._poll_manifest)
        self.get_logger().info(
            f'recon ready: manifest={self.manifest_path}, output={self.output_root}, '
            f'camera tilt={self.get_parameter("camera_forward_tilt_deg").value:.1f} deg forward'
        )

    def _declare_parameters(self):
        values = {
            'manifest_path': '/home/nx163/youth-vision-runtime/overlays/latest_crops/manifest_fast.json',
            'frame_source_path': '/home/nx163/youth-vision-runtime/overlays/latest.jpg',
            'output_root': '/home/nx163/youth-vision-runtime/recon_results/current',
            'global_position_topic': '/mavros/global_position/global',
            'local_pose_topic': '/mavros/local_position/pose',
            'relative_altitude_topic': '/mavros/global_position/rel_alt',
            'gps_raw_topic': '/mavros/gpsstatus/gps1/raw',
            'manifest_poll_hz': 40.0,
            'telemetry_max_gap_sec': 0.35,
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
            'camera_offset_flu_m': [0.0, 0.0, 0.0],
            'ground_altitude_mode': 'home_relative',
            'fixed_ground_altitude_msl_m': 0.0,
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
        self.positions.add(message_time(message), (message.latitude, message.longitude, message.altitude))

    def _pose_callback(self, message):
        q = message.pose.orientation
        self.attitudes.add(message_time(message), (q.x, q.y, q.z, q.w))

    def _relative_altitude_callback(self, message):
        self.relative_altitudes.add(time.time(), float(message.data))

    def _gps_raw_callback(self, message):
        h_acc_m = float(message.h_acc) * 0.001
        if message.h_acc == 0 or message.h_acc == 4294967295:
            h_acc_m = 0.0
        self.rtk_fixes.add(message_time(message), (int(message.fix_type), h_acc_m))

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
        if key == self.last_manifest_key:
            return
        if capture_ns <= 0:
            self.last_manifest_key = key
            self._set_status('vision_timestamp_missing', 'runtime manifest must provide capture_timestamp_unix_ns')
            return
        capture_time = capture_ns * 1e-9
        max_gap = float(self.get_parameter('telemetry_max_gap_sec').value)
        position = self.positions.interpolate(capture_time, lerp_tuple, max_gap)
        attitude = self.attitudes.interpolate(capture_time, quat_slerp, max_gap)
        fix = self.rtk_fixes.nearest(capture_time, max_gap)
        if position is None or attitude is None:
            self._set_status('waiting_for_fcu_telemetry', 'no position/attitude sample aligned to the camera frame')
            return
        if fix is None or fix[0] < int(self.get_parameter('rtk_fixed_min_fix_type').value):
            self._set_status('waiting_for_rtk_fixed', f'GPS fix_type={None if fix is None else fix[0]}')
            return
        if not bool(self.get_parameter('calibration_valid').value):
            self._set_status('waiting_for_camera_calibration', 'calibration_valid is false')
            return
        ground_altitude = self._ground_altitude(capture_time, position[2], max_gap)
        if ground_altitude is None:
            self._set_status('waiting_for_ground_altitude', self.get_parameter('ground_altitude_mode').value)
            return
        frame_width = int(manifest.get('frame_width', 0))
        frame_height = int(manifest.get('frame_height', 0))
        if frame_width <= 0 or frame_height <= 0:
            self.last_manifest_key = key
            self._set_status('vision_geometry_missing', 'frame dimensions absent from manifest')
            return
        # Mark a frame consumed only after matching telemetry and geometry are
        # available. This lets a just-arrived camera frame be retried while its
        # corresponding MAVROS samples are still entering the short buffer.
        self.last_manifest_key = key
        camera = self._camera_model(frame_width, frame_height)
        crop_root = self.manifest_path.parent
        projected = 0
        for crop in manifest.get('crops', []):
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
                label=str(crop.get('class_label', '')),
                class_id=int(crop.get('class_id', -1)),
                confidence=float(crop.get('class_prob', 0.0)),
                pose_score=float(crop.get('pose_score', 0.0)),
                frame_path=str(self.frame_source_path),
                crop_path=str(crop_path),
                horizontal_sigma_m=fix[1],
                center_distance_norm=min(
                    1.0,
                    math.hypot(
                        (float(center[0]) - frame_width * 0.5) / (frame_width * 0.5),
                        (float(center[1]) - frame_height * 0.5) / (frame_height * 0.5),
                    ),
                ),
            )
            track = self.tracks.add(observation)
            self._write_track(track)
            projected += 1
        if projected:
            self._set_status('tracking', f'projected={projected}, tracks={len(self.tracks.tracks)}')
        else:
            self._set_status('waiting_for_target', 'manifest contains no usable target crops')

    def _ground_altitude(self, timestamp, aircraft_altitude, max_gap):
        mode = str(self.get_parameter('ground_altitude_mode').value)
        if mode == 'fixed_msl':
            return float(self.get_parameter('fixed_ground_altitude_msl_m').value)
        if mode == 'home_relative':
            relative = self.relative_altitudes.interpolate(timestamp, lambda a, b, r: a + r * (b - a), max_gap)
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
        target_dir = self.output_root / track.target_id
        frame_destination = target_dir / 'frame.jpg'
        crop_destination = target_dir / 'crop_128.jpg'
        best = fused['best']
        record = {
            'target_id': track.target_id,
            'frame_path': f'{track.target_id}/frame.jpg',
            'crop_path': f'{track.target_id}/crop_128.jpg',
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
            'observation_span_sec': round(fused['observation_span_sec'], 3),
            'coordinate_fusion': {
                'method': 'confidence_pose_image_center_weighted_mean',
                'mean_center_weight': round(fused['mean_center_weight'], 6),
            },
            'rtk_fixed': True,
            'valid': valid,
            'status': status,
        }
        message = ReconTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'wgs84'
        message.target_id = track.target_id
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
        if now - self.last_result_write.get(track.target_id, 0.0) >= result_period:
            atomic_json(target_dir / 'result.json', record)
            self.last_result_write[track.target_id] = now

        snapshot_period = 1.0 / max(1.0, float(self.get_parameter('snapshot_write_hz').value))
        if now - self.last_snapshot_write.get(track.target_id, 0.0) >= snapshot_period:
            source_frame = Path(best.frame_path)
            source_crop = Path(best.crop_path)
            if source_frame.is_file() and source_crop.is_file():
                atomic_copy(source_frame, frame_destination)
                atomic_copy(source_crop, crop_destination)
                self.last_snapshot_write[track.target_id] = now


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
