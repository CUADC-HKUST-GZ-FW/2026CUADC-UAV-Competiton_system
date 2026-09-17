#!/usr/bin/env python3
"""Select one competition target without starting or commanding flight control."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from uav_interfaces.msg import ReconTarget

from competition_selector_core import choose_competition_target, normalize_record


def read_json(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError, TypeError):
        return None


def atomic_json(path, value):
    temporary = path.with_name(f'{path.name}.{os.getpid()}.tmp')
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + '\n',
        encoding='utf-8',
    )
    os.replace(temporary, path)


class CompetitionSelector(Node):
    def __init__(self, args):
        super().__init__('competition_selector')
        self.mode = args.mode
        self.session_root = args.session_root.resolve()
        self.required_targets = args.required_targets
        self.dedup_radius_m = args.dedup_radius_m
        self.settle_sec = args.settle_sec
        self.allow_confirmed = args.allow_confirmed
        self.output_path = self.session_root / 'competition_selected.json'
        self.selection = None
        self.message = None
        self.last_signature = None
        self.last_suppression_signature = None
        self.signature_since = time.monotonic()
        self.last_publish = 0.0
        self.publish_count = 0

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            ReconTarget,
            '/vision/competition_selected_target',
            qos,
        )
        self.timer = self.create_timer(args.poll_period_sec, self.tick)
        self.get_logger().info(
            'Competition selector started '
            f'mode={self.mode} session={self.session_root} '
            f'required_nonempty_targets={self.required_targets} '
            f'distinct_target_distance_m={self.dedup_radius_m:.2f} '
            f'accepted_statuses='
            f'{"finalized,confirmed" if self.allow_confirmed else "finalized"} '
            'flight_command_output=disabled'
        )

    def load_records(self):
        records = []
        for path in sorted(self.session_root.glob('target_*/result.json')):
            record = normalize_record(read_json(path), path.parent.name)
            if record:
                records.append(record)
        return records

    def make_message(self, selected):
        message = ReconTarget()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'wgs84'
        message.target_id = selected['target_id']
        message.frame_path = selected['frame_path']
        message.crop_path = selected['crop_path']
        message.label = selected['label']
        message.class_id = selected['class_id']
        message.confidence = selected['confidence']
        message.latitude = selected['latitude']
        message.longitude = selected['longitude']
        message.altitude_msl_m = selected['altitude_msl_m']
        message.horizontal_radius_95_m = selected['horizontal_radius_95_m']
        message.observation_count = selected['observation_count']
        message.rtk_fixed = selected['rtk_fixed']
        message.valid = True
        message.status = 'finalized'
        return message

    @staticmethod
    def public_record(record):
        return {
            'target_id': record['target_id'],
            'label': record['label'],
            'class_id': record['class_id'],
            'competition_value': record['competition_value'],
            'confidence': round(record['confidence'], 6),
            'label_consensus': round(record['label_consensus'], 6),
            'observation_count': record['observation_count'],
            'frame_path': record['frame_path'],
            'crop_path': record['crop_path'],
            'coordinate': {
                'latitude': round(record['latitude'], 8),
                'longitude': round(record['longitude'], 8),
                'altitude_msl_m': round(record['altitude_msl_m'], 3),
                'horizontal_radius_95_m': round(
                    record['horizontal_radius_95_m'], 3
                ),
            },
            'rtk_fixed': record['rtk_fixed'],
            'valid': record['valid'],
            'status': record['status'],
        }

    @classmethod
    def public_suppressed_record(cls, record):
        public = cls.public_record(record)
        public['selection_reason'] = record.get(
            '_selection_reason',
            'not_selected',
        )
        if record.get('_suppressed_by_target_id'):
            public['suppressed_by_target_id'] = record[
                '_suppressed_by_target_id'
            ]
        if '_distance_m' in record:
            public['distance_m'] = round(record['_distance_m'], 3)
        return public

    def finalize(self, decision, representatives, ignored):
        selected = decision['selected']
        payload = {
            'schema_version': 1,
            'session_id': self.session_root.name,
            'mode': self.mode,
            'ready': True,
            'selected_at_unix_s': time.time(),
            'selection_rule': decision['selection_rule'],
            'required_nonempty_targets': self.required_targets,
            'dedup_radius_m': self.dedup_radius_m,
            'distinct_target_distance_m': self.dedup_radius_m,
            'selected_target': self.public_record(selected),
            'candidates_used': [
                self.public_record(record) for record in decision['candidates']
            ],
            'suppressed_candidates': [
                self.public_suppressed_record(record) for record in ignored
            ],
            # Retained for older dashboard readers.  These are suppressed
            # finalized candidates, not legacy confirmed-only results.
            'ignored_confirmed_candidates': [
                self.public_suppressed_record(record) for record in ignored
            ],
            'eligible_nonempty_targets': len(representatives),
            'confirmed_nonempty_clusters': len(representatives),
            'ros_interface': {
                'topic': '/vision/competition_selected_target',
                'message_type': 'uav_interfaces/msg/ReconTarget',
            },
            'flight_command_published': False,
        }
        atomic_json(self.output_path, payload)
        self.selection = payload
        self.message = self.make_message(selected)
        self.get_logger().info(
            'Competition target locked '
            f'rule={decision["selection_rule"]} '
            f'target_id={selected["target_id"]} '
            f'label={selected["label"]} '
            f'value={selected["competition_value"]} '
            f'lat={selected["latitude"]:.8f} '
            f'lon={selected["longitude"]:.8f} '
            'flight_command_published=false'
        )

    def publish_selected(self):
        now = time.monotonic()
        if self.message is None or now - self.last_publish < 1.0:
            return
        self.message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.message)
        self.last_publish = now
        self.publish_count += 1
        if self.publish_count == 1:
            self.get_logger().info(
                'Published selected reconnaissance result on '
                '/vision/competition_selected_target; no flight command was sent'
            )

    def tick(self):
        if self.selection is not None:
            self.publish_selected()
            return

        decision, representatives, ignored = choose_competition_target(
            self.load_records(),
            self.mode,
            self.required_targets,
            self.dedup_radius_m,
            self.allow_confirmed,
        )
        signature = tuple(
            sorted(
                (
                    item['target_id'],
                    item['label'],
                    item['observation_count'],
                    round(item['latitude'], 8),
                    round(item['longitude'], 8),
                )
                for item in representatives
            )
        )
        if signature != self.last_signature:
            self.last_signature = signature
            self.signature_since = time.monotonic()
            self.get_logger().info(
                f'Competition eligible candidates={len(representatives)}/'
                f'{self.required_targets} ids={signature} '
                f'suppressed={len(ignored)}'
            )
        suppression_signature = tuple(
            sorted(
                (
                    record['target_id'],
                    record.get('_selection_reason', 'not_selected'),
                    record.get('_suppressed_by_target_id', '-'),
                    round(record.get('_distance_m', -1.0), 3),
                )
                for record in ignored
            )
        )
        if suppression_signature != self.last_suppression_signature:
            self.last_suppression_signature = suppression_signature
            for record in ignored:
                self.get_logger().info(
                    'Competition candidate suppressed '
                    f'target_id={record["target_id"]} '
                    f'label={record["label"]} '
                    f'reason={record.get("_selection_reason", "not_selected")} '
                    f'kept={record.get("_suppressed_by_target_id", "-")} '
                    f'distance_m={record.get("_distance_m", float("nan")):.3f}'
                )
        if decision is None:
            return
        if time.monotonic() - self.signature_since < self.settle_sec:
            return
        self.finalize(decision, representatives, ignored)
        self.publish_selected()


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('digit', 'image'), required=True)
    parser.add_argument('--session-root', type=Path, required=True)
    parser.add_argument('--required-targets', type=int, default=3)
    parser.add_argument('--dedup-radius-m', type=float, default=10.0)
    parser.add_argument('--settle-sec', type=float, default=1.0)
    parser.add_argument(
        '--allow-confirmed',
        action='store_true',
        help='Allow legacy confirmed records; intended only for static mode.',
    )
    parser.add_argument('--poll-period-sec', type=float, default=0.25)
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.session_root.is_dir():
        raise SystemExit(f'session root does not exist: {args.session_root}')
    if args.required_targets != 3:
        raise SystemExit('competition mode requires exactly 3 non-empty targets')
    rclpy.init()
    node = CompetitionSelector(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
