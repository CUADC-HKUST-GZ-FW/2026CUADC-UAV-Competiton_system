"""Write /rosout to one chronological file and readable per-node log files."""

import argparse
from datetime import datetime, timezone
import os
import re
import threading

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


_LEVEL_NAMES = {
    10: 'DEBUG',
    20: 'INFO',
    30: 'WARN',
    40: 'ERROR',
    50: 'FATAL',
}


# Known logger names are normalized to the human-facing node file expected in
# this UAV stack.  Unknown loggers are still retained using a sanitized name.
_KNOWN_LOGGER_FILES = (
    ('mavros', 'mavros_node'),
    ('fcu_interface_mavros_node', 'fcu_interface_mavros_node'),
    ('mission_manager_node', 'mission_manager_node'),
    ('payload_monitor_node', 'payload_monitor_node'),
    ('flight_summary_logger_node', 'flight_summary_logger_node'),
    ('sitl_target_gate_node', 'sitl_target_gate_node'),
    ('vision_target_bridge_node', 'vision_target_bridge_node'),
    ('recon_geolocator', 'recon_geolocator'),
)


def _safe_file_stem(logger_name):
    """Turn a ROS logger/node name into a readable, filesystem-safe stem."""
    raw = str(logger_name or '').strip().strip('/')
    if not raw:
        return 'unknown_node'

    lowered = raw.lower()
    for marker, filename in _KNOWN_LOGGER_FILES:
        if marker in lowered:
            return filename

    # Namespaces and sub-loggers remain recognizable while being filesystem-safe.
    stem = raw.replace('/', '__').replace('.', '__')
    stem = re.sub(r'[^A-Za-z0-9_-]+', '_', stem)
    stem = re.sub(r'_+', '_', stem).strip('_')
    return stem or 'unknown_node'


def _stamp_text(msg):
    sec = int(msg.stamp.sec)
    nanosec = int(msg.stamp.nanosec)
    if sec > 0:
        value = sec + nanosec / 1_000_000_000.0
        dt = datetime.fromtimestamp(value, tz=timezone.utc).astimezone()
    else:
        dt = datetime.now(timezone.utc).astimezone()
    return dt.strftime('%Y-%m-%d %H:%M:%S.') + f'{dt.microsecond // 1000:03d}'


def _format_record(msg):
    timestamp = _stamp_text(msg)
    level = _LEVEL_NAMES.get(int(msg.level), str(int(msg.level)))
    logger_name = str(msg.name or 'unknown_node')
    text = str(msg.msg or '')

    lines = text.splitlines() or ['']
    first = f'[{timestamp}] [{level:<5}] [{logger_name}] {lines[0]}'
    if len(lines) == 1:
        return first

    continuation = '\n'.join(f'    {line}' for line in lines[1:])
    return first + '\n' + continuation


class RosoutLogCollector(Node):
    """Persist /rosout in chronological and per-node human-readable forms."""

    def __init__(self, all_nodes_log, node_log_dir):
        super().__init__('uav_rosout_log_collector')

        self.all_nodes_log = os.path.abspath(os.path.expanduser(all_nodes_log))
        self.node_log_dir = os.path.abspath(os.path.expanduser(node_log_dir))
        os.makedirs(os.path.dirname(self.all_nodes_log), exist_ok=True)
        os.makedirs(self.node_log_dir, exist_ok=True)

        self._lock = threading.Lock()
        self._node_streams = {}
        self._all_stream = open(
            self.all_nodes_log,
            'a',
            encoding='utf-8',
            buffering=1,
        )

        rosout_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
        )
        self.create_subscription(
            Log,
            '/rosout',
            self._rosout_callback,
            rosout_qos,
        )

        self.get_logger().info(
            'named ROS log collector started '
            f'all_nodes_log={self.all_nodes_log} '
            f'node_log_dir={self.node_log_dir}'
        )

    def _stream_for_logger(self, logger_name):
        stem = _safe_file_stem(logger_name)
        stream = self._node_streams.get(stem)
        if stream is not None:
            return stream

        path = os.path.join(self.node_log_dir, f'{stem}.log')
        stream = open(path, 'a', encoding='utf-8', buffering=1)
        self._node_streams[stem] = stream
        return stream

    def _rosout_callback(self, msg):
        record = _format_record(msg)
        with self._lock:
            self._all_stream.write(record + '\n')
            self._all_stream.flush()

            stream = self._stream_for_logger(msg.name)
            stream.write(record + '\n')
            stream.flush()

    def destroy_node(self):
        with self._lock:
            for stream in self._node_streams.values():
                try:
                    stream.close()
                except Exception:
                    pass
            self._node_streams.clear()
            try:
                self._all_stream.close()
            except Exception:
                pass
        return super().destroy_node()


def _parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--all-nodes-log', required=True)
    parser.add_argument('--node-log-dir', required=True)
    return parser.parse_args()


def main(args=None):
    cli = _parse_args()
    rclpy.init(args=args)
    node = RosoutLogCollector(
        all_nodes_log=cli.all_nodes_log,
        node_log_dir=cli.node_log_dir,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
