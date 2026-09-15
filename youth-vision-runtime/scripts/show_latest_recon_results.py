#!/usr/bin/env python3
"""Show the latest reconnaissance results with spatial de-duplication."""

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import sys
import time


DEFAULT_ROOT = Path('/home/nx163/youth-vision-runtime/recon_results')
EARTH_RADIUS_M = 6378137.0
HIDDEN_LABELS = frozenset({'empty', '空标靶'})


def read_json(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError, TypeError):
        return None


def result_paths(session):
    return sorted(session.glob('target_*/result.json')) if session.is_dir() else []


def select_session(root, explicit_session=None, latest_only=False):
    if explicit_session:
        session = Path(explicit_session).expanduser().resolve()
        return session, False, session

    latest_link = root / 'latest'
    latest = latest_link.resolve() if latest_link.exists() else latest_link
    if result_paths(latest) or latest_only:
        return latest, False, latest

    sessions_root = root / 'sessions'
    candidates = []
    if sessions_root.is_dir():
        for session in sessions_root.iterdir():
            paths = result_paths(session)
            if paths:
                newest = max(path.stat().st_mtime for path in paths)
                candidates.append((newest, session))
    if candidates:
        candidates.sort(reverse=True, key=lambda item: item[0])
        return candidates[0][1], True, latest
    return latest, False, latest


def normalize_record(path):
    raw = read_json(path)
    if not isinstance(raw, dict):
        return None
    coordinate = raw.get('coordinate') or {}
    recognition = raw.get('recognition') or {}
    try:
        latitude = float(coordinate['latitude'])
        longitude = float(coordinate['longitude'])
        altitude = float(coordinate.get('altitude_msl_m', 0.0))
        confidence = float(recognition.get('confidence', 0.0))
        radius95 = float(coordinate.get('horizontal_radius_95_m', 0.0))
    except (KeyError, TypeError, ValueError):
        return None
    if not (-90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
        return None
    if not all(math.isfinite(value) for value in (latitude, longitude, altitude, confidence, radius95)):
        return None
    navigation = raw.get('navigation') or {}
    session_root = path.parent.parent.resolve()

    def artifact_path(value, fallback):
        candidate = (session_root / str(value or fallback)).resolve()
        if not candidate.is_relative_to(session_root):
            return ''
        return str(candidate) if candidate.is_file() else ''

    return {
        'target_id': str(raw.get('target_id') or path.parent.name),
        'label': str(recognition.get('label', 'unknown')),
        'class_id': int(recognition.get('class_id', -1)),
        'confidence': max(0.0, min(1.0, confidence)),
        'label_consensus': float(recognition.get('label_consensus', 0.0)),
        'latitude': latitude,
        'longitude': longitude,
        'altitude_msl_m': altitude,
        'horizontal_radius_95_m': max(0.0, radius95),
        'observation_count': int(raw.get('observation_count', 0)),
        'status': str(raw.get('status', 'unknown')),
        'valid': bool(raw.get('valid', False)),
        'rtk_fixed': bool(raw.get('rtk_fixed', navigation.get('rtk_fixed', False))),
        'gps_fix_type': int(navigation.get('gps_fix_type', 0)),
        'attitude_source': str(navigation.get('attitude_source', 'unknown')),
        'crop_path': artifact_path(raw.get('crop_path'), f'{path.parent.name}/crop_128.jpg'),
        'frame_path': artifact_path(raw.get('frame_path'), f'{path.parent.name}/frame.jpg'),
        'result_path': str(path),
    }


def distance_m(a, b):
    lat1 = math.radians(a['latitude'])
    lat2 = math.radians(b['latitude'])
    dlat = lat2 - lat1
    dlon = math.radians(b['longitude'] - a['longitude'])
    value = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(value)))


def is_hidden_label(label):
    return str(label).strip().casefold() in HIDDEN_LABELS


def is_confirmed(record):
    return record['valid'] and record['status'] in {'confirmed', 'finalized'}


def cluster_records(records, radius_m):
    """Merge only confirmed same-label records without transitive chaining."""
    confirmed = sorted(
        (record for record in records if is_confirmed(record)),
        key=lambda record: (
            record['label'],
            -record['observation_count'],
            -record['confidence'],
            record['target_id'],
        ),
    )
    groups = []
    for record in confirmed:
        compatible = [
            group
            for group in groups
            if group[0]['label'] == record['label']
            and all(distance_m(record, member) <= radius_m for member in group)
        ]
        if compatible:
            group = min(
                compatible,
                key=lambda candidate: max(distance_m(record, member) for member in candidate),
            )
            group.append(record)
        else:
            groups.append([record])

    # Pending records remain visible for diagnosis, but never influence another
    # record's coordinate, label, validity, or uncertainty.
    groups.extend([record] for record in records if not is_confirmed(record))
    return groups


def member_weight(record):
    confidence = max(0.05, record['confidence'])
    observations = max(1.0, math.sqrt(max(1, record['observation_count'])))
    uncertainty = max(1.0, record['horizontal_radius_95_m'])
    validity = 2.0 if record['valid'] else 1.0
    return confidence * observations * validity / (uncertainty * uncertainty)


def summarize_group(group):
    weights = [member_weight(record) for record in group]
    total_weight = sum(weights)
    latitude = sum(weight * record['latitude'] for weight, record in zip(weights, group)) / total_weight
    longitude = sum(weight * record['longitude'] for weight, record in zip(weights, group)) / total_weight
    altitude = statistics.median(record['altitude_msl_m'] for record in group)

    label_votes = {}
    for weight, record in zip(weights, group):
        label_votes[record['label']] = label_votes.get(record['label'], 0.0) + weight
    label = max(label_votes, key=label_votes.get)
    label_members = [record for record in group if record['label'] == label]
    representative = max(
        label_members,
        key=lambda record: (
            record['valid'],
            record['status'] in {'confirmed', 'finalized'},
            record['observation_count'],
            record['confidence'],
        ),
    )
    image_record = max(
        label_members,
        key=lambda record: (record['confidence'], record['valid'], record['observation_count']),
    )
    center = {'latitude': latitude, 'longitude': longitude}
    spread = max((distance_m(center, record) for record in group), default=0.0)
    aliases = sorted(label_votes, key=label_votes.get, reverse=True)
    return {
        'target_id': representative['target_id'],
        'merged_target_ids': sorted(record['target_id'] for record in group),
        'merged_count': len(group),
        'label': label,
        'alternative_labels': [item for item in aliases if item != label],
        'confidence': max(record['confidence'] for record in label_members),
        'label_consensus': label_votes[label] / sum(label_votes.values()),
        'latitude': latitude,
        'longitude': longitude,
        'altitude_msl_m': altitude,
        'cluster_spread_m': spread,
        'horizontal_radius_95_m': max(record['horizontal_radius_95_m'] for record in group),
        'observation_count': sum(record['observation_count'] for record in group),
        'status': (
            'finalized'
            if any(record['status'] == 'finalized' for record in group)
            else 'confirmed'
            if any(record['status'] == 'confirmed' for record in group)
            else representative['status']
        ),
        'valid': any(record['valid'] for record in group),
        'rtk_fixed': all(record['rtk_fixed'] for record in group),
        'gps_fix_type': min(record['gps_fix_type'] for record in group),
        'attitude_source': representative['attitude_source'],
        'representative_crop_path': image_record['crop_path'],
        'representative_frame_path': image_record['frame_path'],
    }


def load_summary(root, session_arg, latest_only, dedup_radius_m):
    session, fallback, latest = select_session(root, session_arg, latest_only)
    records = []
    invalid_files = []
    for path in result_paths(session):
        record = normalize_record(path)
        if record is None:
            invalid_files.append(str(path))
        else:
            records.append(record)
    records = [record for record in records if not is_hidden_label(record['label'])]
    summaries = [summarize_group(group) for group in cluster_records(records, dedup_radius_m)]
    summaries.sort(
        key=lambda item: (item['confidence'], item['observation_count']),
        reverse=True,
    )
    status = read_json(session / 'status.json') or {}
    latest_status = read_json(latest / 'status.json') or {}
    return {
        'session': str(session),
        'session_name': session.name,
        'fallback_to_previous_session': fallback,
        'latest_session': str(latest),
        'pipeline_status': status,
        'latest_pipeline_status': latest_status,
        'dedup_radius_m': dedup_radius_m,
        'raw_result_count': len(records),
        'deduplicated_count': len(summaries),
        'invalid_result_files': invalid_files,
        'targets': summaries,
    }


def format_timestamp(value):
    try:
        return datetime.fromtimestamp(float(value)).strftime('%Y-%m-%d %H:%M:%S')
    except (TypeError, ValueError, OSError):
        return '未知'


def status_text(status):
    names = {
        'finalized': '已定稿',
        'confirmed': '已确认',
        'collecting_observations': '采集中',
        'tracking': '跟踪中',
        'waiting_for_target': '等待目标',
        'waiting_for_gps_fix': '等待GPS定位',
        'waiting_for_fcu_telemetry': '等待飞控遥测',
        'waiting_for_vision': '等待图像识别',
    }
    return names.get(status, status or '未知')


def print_human(summary):
    line = '=' * 78
    print(line)
    print('侦察识别结果汇总')
    print(line)
    print(f"任务: {summary['session_name']}")
    if summary['fallback_to_previous_session']:
        print('提示: 当前任务没有目标结果，以下为最近一次有结果的任务。')
        latest = summary['latest_pipeline_status']
        print(f"当前任务状态: {status_text(latest.get('status'))} | {latest.get('detail', '')}")
    status = summary['pipeline_status']
    print(
        f"任务状态: {status_text(status.get('status'))} | "
        f"更新时间: {format_timestamp(status.get('timestamp_unix_s'))}"
    )
    if status.get('detail'):
        print(f"状态说明: {status['detail']}")
    print(
        f"去重统计: 原始 {summary['raw_result_count']} 条 -> "
        f"目标 {summary['deduplicated_count']} 个 | 半径 {summary['dedup_radius_m']:.1f} m"
    )
    if summary['invalid_result_files']:
        print(f"警告: {len(summary['invalid_result_files'])} 个结果文件无法解析。")
    print('-' * 78)

    if not summary['targets']:
        print('尚未发现可读取的标靶坐标结果。')
        return

    for index, target in enumerate(summary['targets'], 1):
        state = status_text(target['status'])
        validity = '有效' if target['valid'] else '待确认'
        rtk = 'RTK Fixed' if target['rtk_fixed'] else f"GPS fix {target['gps_fix_type']}"
        print(
            f"[{index:02d}] 识别结果: {target['label']}  |  "
            f"置信度: {target['confidence'] * 100:6.2f}%  |  {state}/{validity}"
        )
        print(
            f"     标靶中心: 纬度 {target['latitude']:.8f}, "
            f"经度 {target['longitude']:.8f}, 海拔 {target['altitude_msl_m']:.2f} m"
        )
        print(
            f"     观测: {target['observation_count']} 帧  |  {rtk}  |  "
            f"坐标95%半径: {target['horizontal_radius_95_m']:.2f} m  |  "
            f"组内离散: {target['cluster_spread_m']:.2f} m"
        )
        if target['merged_count'] > 1:
            print(
                f"     疑似重复: 已合并 {target['merged_count']} 条 "
                f"({', '.join(target['merged_target_ids'])})"
            )
        if target['alternative_labels']:
            print(
                f"     标签冲突: 主标签 {target['label']}，其他曾识别为 "
                f"{', '.join(target['alternative_labels'])}"
            )
        print('-' * 78)


def parse_args():
    parser = argparse.ArgumentParser(description='读取、去重并显示最近的标靶识别和坐标结果。')
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT, help='recon_results 根目录')
    parser.add_argument('--session', help='指定任务目录；默认读取 latest')
    parser.add_argument('--latest-only', action='store_true', help='当前任务无结果时不回退到历史任务')
    parser.add_argument('--dedup-radius-m', type=float, default=8.0, help='疑似相同标靶的合并半径，默认8米')
    parser.add_argument('--json', action='store_true', help='输出机器可读JSON')
    parser.add_argument('--watch', nargs='?', const=1.0, type=float, help='持续刷新，可指定刷新秒数')
    return parser.parse_args()


def main():
    args = parse_args()
    if args.dedup_radius_m <= 0:
        raise SystemExit('--dedup-radius-m 必须大于0')
    if args.watch is not None and args.watch <= 0:
        raise SystemExit('--watch 刷新时间必须大于0')

    while True:
        summary = load_summary(args.root, args.session, args.latest_only, args.dedup_radius_m)
        if args.watch is not None and sys.stdout.isatty():
            print('\033[2J\033[H', end='')
        if args.json:
            print(json.dumps(summary, ensure_ascii=False, indent=2))
        else:
            print_human(summary)
        if args.watch is None:
            break
        time.sleep(args.watch)


if __name__ == '__main__':
    main()
