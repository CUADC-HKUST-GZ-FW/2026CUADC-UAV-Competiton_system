"""ROS 2 node: MAVROS aircraft position + recognition JSONL -> overlay, log, optional video.

Aircraft lat/lon/alt for each recorded MP4 frame is written to a sibling JSONL
sidecar. This node does not project pixels to ground. The optional TargetCommand
publisher is kept so uav_ros2_project can still subscribe to /vision/target_command.
"""

import json
import os
import time
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from .overlay import draw_geo_overlay
from .recognition import RecognitionTail, detection_pixel, record_wall_time
from .recorder import FrameSidecar, OverlayRecorder, sidecar_path_for_video
from .telemetry import TelemetryBuffer


class GeoBridgeNode(Node):
    def __init__(self):
        super().__init__('geo_bridge_node')
        self._declare()
        self._load()

        self.telemetry = TelemetryBuffer()
        self.tail = RecognitionTail(self.recognition_log)
        self.recorder = None
        self.sidecar = None
        if self.record_video:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            os.makedirs(self.record_dir, exist_ok=True)
            path = os.path.join(self.record_dir, f'geo_overlay_{stamp}.mp4')
            self.recorder = OverlayRecorder(path, fps=self.record_fps)
            sidecar_path = sidecar_path_for_video(path)
            self.sidecar = FrameSidecar(sidecar_path)
            self.get_logger().info(
                f'[GEO] recording enabled path={path} sidecar={sidecar_path}'
            )

        os.makedirs(os.path.dirname(self.fused_log) or '.', exist_ok=True)
        self._fused = open(self.fused_log, 'a', encoding='utf-8')
        self._overlay_mtime = None
        self._last_targets = []
        self._last_match_age = None
        self._last_publish = 0.0
        self._last_status = 0.0
        self._warned_no_target_geo = False
        self.fused_count = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Float64, self.rel_alt_topic, self._rel_alt_cb, sensor_qos)
        self.create_subscription(NavSatFix, self.global_topic, self._global_cb, sensor_qos)

        self.target_pub = None
        if self.publish_target_command:
            from uav_interfaces.msg import TargetCommand
            self._TargetCommand = TargetCommand
            self.target_pub = self.create_publisher(TargetCommand, self.target_topic, 10)
            self.get_logger().warn(
                '[GEO] publish_target_command is enabled, but this node no longer '
                'solves target WGS84 from pixels; TargetCommand is only forwarded '
                'when a recognition record already contains latitude/longitude'
            )

        self.create_timer(0.02, self._poll_recognition)
        self.create_timer(self.overlay_period_sec, self._update_overlay)
        self.create_timer(self.status_period_sec, self._print_status)
        self.get_logger().info(
            '[GEO] bridge started '
            f'recognition_log={self.recognition_log} '
            f'rel_alt_topic={self.rel_alt_topic} '
            f'global_topic={self.global_topic} '
            f'publish_target_command={str(self.publish_target_command).lower()}'
        )

    def _declare(self):
        defaults = {
            'recognition_log': '/home/nx163/youth-vision-runtime/logs/recognition_events_latest.jsonl',
            'overlay_jpeg': '/home/nx163/youth-vision-runtime/overlays/latest.jpg',
            'annotated_jpeg': '/home/nx163/youth-vision-runtime/overlays/latest_geo.jpg',
            'fused_log': '/home/nx163/youth-vision-runtime/logs/geo_bridge/fused_latest.jsonl',
            'rel_alt_topic': '/mavros/global_position/rel_alt',
            'global_topic': '/mavros/global_position/global',
            'max_match_age_sec': 0.25,
            'overlay_period_sec': 0.05,
            'status_period_sec': 2.0,
            'record_video': True,
            'record_dir': '/home/nx163/youth-vision-runtime/logs/geo_recordings',
            'record_fps': 15.0,
            'draw_hud': False,
            'publish_target_command': False,
            'target_topic': '/vision/target_command',
            'min_class_prob': 0.60,
            'publish_period_sec': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load(self):
        g = self.get_parameter
        self.recognition_log = str(g('recognition_log').value)
        self.overlay_jpeg = str(g('overlay_jpeg').value)
        self.annotated_jpeg = str(g('annotated_jpeg').value)
        self.fused_log = str(g('fused_log').value)
        self.rel_alt_topic = str(g('rel_alt_topic').value)
        self.global_topic = str(g('global_topic').value)
        self.max_match_age_sec = float(g('max_match_age_sec').value)
        self.overlay_period_sec = float(g('overlay_period_sec').value)
        self.status_period_sec = float(g('status_period_sec').value)
        self.record_video = bool(g('record_video').value)
        self.record_dir = str(g('record_dir').value)
        self.record_fps = float(g('record_fps').value)
        self.draw_hud = bool(g('draw_hud').value)
        self.publish_target_command = bool(g('publish_target_command').value)
        self.target_topic = str(g('target_topic').value)
        self.min_class_prob = float(g('min_class_prob').value)
        self.publish_period_sec = float(g('publish_period_sec').value)

    def _now(self):
        return time.time()

    def _rel_alt_cb(self, msg):
        self.telemetry.update_rel_alt(self._now(), msg.data)

    def _global_cb(self, msg):
        self.telemetry.update_global(self._now(), msg.latitude, msg.longitude, msg.altitude)

    def _collect_targets(self, record):
        targets = []
        detections = record.get('detections') or [record]
        for index, item in enumerate(detections):
            lat = item.get('latitude', record.get('latitude'))
            lon = item.get('longitude', record.get('longitude'))
            heading = item.get('heading_deg', record.get('heading_deg'))
            targets.append(
                {
                    'label': str(item.get('class_label', record.get('class_label', '?'))),
                    'prob': item.get('class_prob', record.get('class_prob')),
                    'pixel': detection_pixel(record, index),
                    'latitude': lat,
                    'longitude': lon,
                    'heading_deg': heading,
                }
            )
        return targets

    def _poll_recognition(self):
        for record in self.tail.poll():
            wall = record_wall_time(record) or self._now()
            sample, age = self.telemetry.nearest(wall, max_age_sec=1.0e9)
            self._last_match_age = age
            self._last_targets = self._collect_targets(record)
            self._write_fused(record, sample, age, self._last_targets)
            self._maybe_publish(self._last_targets)

    def _write_fused(self, record, sample, age, targets):
        payload = {
            'record_type': 'fused',
            'timestamp_unix_ms': record.get('timestamp_unix_ms'),
            'frame': record.get('frame'),
            'class_label': record.get('class_label'),
            'class_prob': record.get('class_prob'),
            'match_age_sec': age,
            'relative_alt_m': None if sample is None else sample.relative_alt_m,
            'amsl_m': None if sample is None else sample.amsl_m,
            'aircraft_lat': None if sample is None else sample.latitude,
            'aircraft_lon': None if sample is None else sample.longitude,
            'targets': [
                {
                    'label': target['label'],
                    'prob': target['prob'],
                    'pixel': target['pixel'],
                    'latitude': target.get('latitude'),
                    'longitude': target.get('longitude'),
                    'heading_deg': target.get('heading_deg'),
                }
                for target in targets
            ],
        }
        self._fused.write(json.dumps(payload, ensure_ascii=False) + '\n')
        self._fused.flush()
        self.fused_count += 1

    def _maybe_publish(self, targets):
        if self.target_pub is None or not targets:
            return
        now = self._now()
        if now - self._last_publish < self.publish_period_sec:
            return
        if self._last_match_age is None or abs(self._last_match_age) > self.max_match_age_sec:
            return
        best = max(targets, key=lambda item: float(item.get('prob') or 0.0))
        if best.get('prob') is None or float(best['prob']) < self.min_class_prob:
            return
        if best.get('latitude') is None or best.get('longitude') is None:
            if not self._warned_no_target_geo:
                self.get_logger().warn(
                    '[GEO] recognition has no target lat/lon; '
                    'not publishing /vision/target_command'
                )
                self._warned_no_target_geo = True
            return
        msg = self._TargetCommand()
        msg.latitude = float(best['latitude'])
        msg.longitude = float(best['longitude'])
        msg.heading_deg = float(best['heading_deg'] or 0.0)
        self.target_pub.publish(msg)
        self._last_publish = now
        self.get_logger().info(
            '[GEO] target published '
            f'label={best["label"]} lat={msg.latitude:.7f} lon={msg.longitude:.7f} '
            f'heading_deg={msg.heading_deg:.1f}'
        )

    def _update_overlay(self):
        if not os.path.exists(self.overlay_jpeg):
            return
        mtime = os.path.getmtime(self.overlay_jpeg)
        if self._overlay_mtime is not None and mtime <= self._overlay_mtime:
            return
        frame = cv2.imread(self.overlay_jpeg)
        if frame is None:
            return
        self._overlay_mtime = mtime
        now = self._now()
        sample = self.telemetry.latest
        age = None if sample.wall_time <= 0.0 else now - sample.wall_time
        output = frame
        if self.draw_hud:
            output = draw_geo_overlay(frame.copy(), sample, self._last_targets, age)
        os.makedirs(os.path.dirname(self.annotated_jpeg) or '.', exist_ok=True)
        cv2.imwrite(self.annotated_jpeg, output)
        if self.recorder is not None:
            frame_index = self.recorder.write(output)
            if frame_index is not None and self.sidecar is not None:
                self.sidecar.append(frame_index, sample, now=now)

    def _print_status(self):
        sample = self.telemetry.latest
        age = self.telemetry.age(self._now())
        rec = 'off' if self.recorder is None else f'{self.recorder.frames}f'
        sidecar_n = 'off' if self.sidecar is None else f'{self.sidecar.rows}'
        self.get_logger().info(
            '[GEO][STATUS] '
            f'lat={sample.latitude} '
            f'lon={sample.longitude} '
            f'rel_alt={sample.relative_alt_m} '
            f'amsl={sample.amsl_m} '
            f'pos_age_s={None if age is None else round(age, 3)} '
            f'rel_alt_n={self.telemetry.rel_alt_count} '
            f'global_n={self.telemetry.global_count} '
            f'fused={self.fused_count} '
            f'targets={len(self._last_targets)} '
            f'record={rec} '
            f'sidecar={sidecar_n}'
        )

    def destroy_node(self):
        self.tail.close()
        if self._fused is not None:
            self._fused.close()
        sidecar_summary = None
        if self.sidecar is not None:
            sidecar_summary = self.sidecar.summary()
            self.sidecar.close()
        if self.recorder is not None:
            summary = self.recorder.summary()
            self.recorder.close()
            sidecar_rows = None if sidecar_summary is None else sidecar_summary['rows']
            self.get_logger().info(
                '[GEO] recording finalized '
                f'path={summary["path"]} frames={summary["frames"]} '
                f'sidecar={None if sidecar_summary is None else sidecar_summary["path"]} '
                f'sidecar_rows={sidecar_rows}'
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GeoBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
