"""Read-only standalone logger for confirmed servo openings and aircraft GPS."""

import csv
from datetime import datetime, timezone
import json
import math
import os

from mavros_msgs.msg import RCOut
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus

from .servo_open_detector import ServoOpenDetector, ServoOpenDetectorConfig


class ServoOpenLoggerNode(Node):
    """Record the first and confirming GPS samples for every servo opening."""

    CSV_FIELDS = (
        'event_time',
        'servo_channel',
        'first_pwm',
        'confirmed_pwm',
        'sample_count',
        'confirmation_duration_ms',
        'open_latitude',
        'open_longitude',
        'open_altitude_m',
        'open_gps_age_ms',
        'open_gps_valid',
        'confirmed_latitude',
        'confirmed_longitude',
        'confirmed_altitude_m',
        'confirmed_gps_age_ms',
        'confirmed_gps_valid',
    )

    def __init__(self):
        super().__init__('servo_open_logger_node')
        self.declare_parameter('servo_channel', 7)
        self.declare_parameter('release_pwm', 1900)
        self.declare_parameter('safe_pwm', 1350)
        self.declare_parameter('pwm_tolerance_us', 20)
        self.declare_parameter('required_open_samples', 3)
        self.declare_parameter('required_closed_samples', 3)
        self.declare_parameter('gps_stale_timeout_s', 1.0)
        self.declare_parameter('rc_out_topic', '/mavros/rc/out')
        self.declare_parameter('gps_topic', '/mavros/global_position/global')
        self.declare_parameter(
            'output_root',
            os.path.expanduser('~/uav_flight_logs/servo_open_test'),
        )

        self.servo_channel = int(self.get_parameter('servo_channel').value)
        if not 1 <= self.servo_channel <= 16:
            raise ValueError('servo_channel must be within [1, 16]')
        self.gps_stale_timeout_s = float(
            self.get_parameter('gps_stale_timeout_s').value
        )
        if not math.isfinite(self.gps_stale_timeout_s) or self.gps_stale_timeout_s <= 0:
            raise ValueError('gps_stale_timeout_s must be a finite value > 0')

        config = ServoOpenDetectorConfig(
            release_pwm=int(self.get_parameter('release_pwm').value),
            safe_pwm=int(self.get_parameter('safe_pwm').value),
            pwm_tolerance_us=int(self.get_parameter('pwm_tolerance_us').value),
            required_open_samples=int(
                self.get_parameter('required_open_samples').value
            ),
            required_closed_samples=int(
                self.get_parameter('required_closed_samples').value
            ),
        )
        self.detector = ServoOpenDetector(config)
        self.last_gps = None
        self.last_gps_received_at = None
        self.candidate_gps = None
        self.candidate_first_pwm = None

        self.run_dir = self._make_run_directory(
            str(self.get_parameter('output_root').value)
        )
        self.csv_path = os.path.join(self.run_dir, 'servo_open_events.csv')
        self._write_manifest(config)
        with open(self.csv_path, 'w', encoding='utf-8', newline='') as stream:
            csv.DictWriter(stream, fieldnames=self.CSV_FIELDS).writeheader()

        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter('gps_topic').value),
            self._gps_callback,
            sensor_qos,
        )
        self.create_subscription(
            RCOut,
            str(self.get_parameter('rc_out_topic').value),
            self._rc_out_callback,
            sensor_qos,
        )
        self.get_logger().info(
            '[SERVO_TEST][state=WAITING_FOR_CLOSED] read-only logger started '
            f'channel={self.servo_channel} safe_pwm={config.safe_pwm} '
            f'release_pwm={config.release_pwm} output={self.csv_path}'
        )

    def _now(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    @staticmethod
    def _wall_time():
        return datetime.now(timezone.utc).astimezone().isoformat(
            timespec='milliseconds'
        )

    def _make_run_directory(self, configured_root):
        root = os.path.abspath(os.path.expanduser(configured_root))
        os.makedirs(root, exist_ok=True)
        base = 'servo_open_' + datetime.now().astimezone().strftime(
            '%Y%m%d_%H%M%S_%f'
        )[:-3]
        candidate = os.path.join(root, base)
        suffix = 0
        while True:
            try:
                os.makedirs(candidate, exist_ok=False)
                return candidate
            except FileExistsError:
                suffix += 1
                candidate = os.path.join(root, f'{base}_{suffix:02d}')

    def _write_manifest(self, config):
        manifest = {
            'schema_version': 1,
            'started_at_local': self._wall_time(),
            'read_only': True,
            'servo_channel': self.servo_channel,
            'release_pwm': config.release_pwm,
            'safe_pwm': config.safe_pwm,
            'pwm_tolerance_us': config.pwm_tolerance_us,
            'required_open_samples': config.required_open_samples,
            'required_closed_samples': config.required_closed_samples,
            'gps_stale_timeout_s': self.gps_stale_timeout_s,
            'rc_out_topic': str(self.get_parameter('rc_out_topic').value),
            'gps_topic': str(self.get_parameter('gps_topic').value),
            'csv_path': self.csv_path,
        }
        path = os.path.join(self.run_dir, 'run_manifest.json')
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(manifest, stream, ensure_ascii=False, indent=2)
            stream.write('\n')

    def _gps_callback(self, msg):
        self.last_gps = msg
        self.last_gps_received_at = self._now()

    def _gps_sample(self, now):
        if self.last_gps is None or self.last_gps_received_at is None:
            return self._empty_gps_sample()
        age_s = max(0.0, now - self.last_gps_received_at)
        finite = all(math.isfinite(float(value)) for value in (
            self.last_gps.latitude,
            self.last_gps.longitude,
            self.last_gps.altitude,
        ))
        valid = (
            finite
            and self.last_gps.status.status >= NavSatStatus.STATUS_FIX
            and age_s <= self.gps_stale_timeout_s
        )
        return {
            'latitude': float(self.last_gps.latitude) if finite else None,
            'longitude': float(self.last_gps.longitude) if finite else None,
            'altitude_m': float(self.last_gps.altitude) if finite else None,
            'age_ms': age_s * 1000.0,
            'valid': valid,
        }

    @staticmethod
    def _empty_gps_sample():
        return {
            'latitude': None,
            'longitude': None,
            'altitude_m': None,
            'age_ms': None,
            'valid': False,
        }

    def _rc_out_callback(self, msg):
        channel_index = self.servo_channel - 1
        if len(msg.channels) <= channel_index:
            self.get_logger().warning(
                '[SERVO_TEST] configured channel unavailable '
                f'channel={self.servo_channel} received_channels={len(msg.channels)}'
            )
            return
        now = self._now()
        for event in self.detector.observe(msg.channels[channel_index], now):
            self._handle_event(event, now)

    def _handle_event(self, event, now):
        if event.kind == 'armed':
            self.get_logger().info(
                '[SERVO_TEST][state=ARMED] closed PWM confirmed; waiting for opening'
            )
            return
        if event.kind == 'rearmed':
            self.candidate_gps = None
            self.candidate_first_pwm = None
            self.get_logger().info(
                '[SERVO_TEST][state=ARMED] close confirmed; detector rearmed'
            )
            return
        if event.kind == 'candidate_started':
            self.candidate_gps = self._gps_sample(now)
            self.candidate_first_pwm = event.pwm
            self.get_logger().info(
                '[SERVO_TEST][state=CANDIDATE] release PWM first observed '
                f'pwm={event.pwm}'
            )
            return
        if event.kind == 'candidate_rejected':
            self.get_logger().warning(
                '[SERVO_TEST][state=ARMED] transient release candidate rejected '
                f'matched_samples={event.sample_count} pwm={event.pwm}'
            )
            self.candidate_gps = None
            self.candidate_first_pwm = None
            return
        if event.kind == 'open_confirmed':
            self._record_open(event, self._gps_sample(now))

    def _record_open(self, event, confirmed_gps):
        open_gps = self.candidate_gps or self._empty_gps_sample()
        row = {
            'event_time': self._wall_time(),
            'servo_channel': self.servo_channel,
            'first_pwm': self.candidate_first_pwm,
            'confirmed_pwm': event.pwm,
            'sample_count': event.sample_count,
            'confirmation_duration_ms': round(event.duration_s * 1000.0, 3),
            'open_latitude': open_gps['latitude'],
            'open_longitude': open_gps['longitude'],
            'open_altitude_m': open_gps['altitude_m'],
            'open_gps_age_ms': open_gps['age_ms'],
            'open_gps_valid': open_gps['valid'],
            'confirmed_latitude': confirmed_gps['latitude'],
            'confirmed_longitude': confirmed_gps['longitude'],
            'confirmed_altitude_m': confirmed_gps['altitude_m'],
            'confirmed_gps_age_ms': confirmed_gps['age_ms'],
            'confirmed_gps_valid': confirmed_gps['valid'],
        }
        with open(self.csv_path, 'a', encoding='utf-8', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=self.CSV_FIELDS)
            writer.writerow(row)
            stream.flush()
        self.get_logger().info(
            '[SERVO_TEST][state=OPEN_CONFIRMED] '
            f'channel={self.servo_channel} pwm={event.pwm} '
            f'samples={event.sample_count} '
            f'duration_ms={event.duration_s * 1000.0:.1f} '
            f'open_lat={open_gps["latitude"]} open_lon={open_gps["longitude"]} '
            f'open_gps_valid={str(open_gps["valid"]).lower()}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ServoOpenLoggerNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
