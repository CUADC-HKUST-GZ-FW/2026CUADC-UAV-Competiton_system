#!/usr/bin/env python3
"""Pure selection logic for the four-target competition reconnaissance mode."""

from __future__ import annotations

import math


EARTH_RADIUS_M = 6378137.0
EMPTY_LABELS = {'empty', '空', '空标靶', 'none'}
IMAGE_VALUES = {
    '机枪兵': 1,
    '火箭兵': 2,
    '多旋翼': 3,
    '固定翼': 4,
    '卡车': 5,
    '防空炮': 6,
    '坦克': 7,
    '直升机': 8,
    '战斗机': 9,
    '运输机': 10,
    '侦察机': 11,
    '轰炸机': 12,
}


def distance_m(left, right):
    lat1 = math.radians(left['latitude'])
    lat2 = math.radians(right['latitude'])
    dlat = lat2 - lat1
    dlon = math.radians(right['longitude'] - left['longitude'])
    value = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(value)))


def mode_value(label, mode):
    text = str(label).strip()
    if text.lower() in EMPTY_LABELS:
        return None
    if mode == 'digit':
        if not text.isdigit() or len(text) > 2:
            return None
        value = int(text)
        return value if 0 <= value <= 99 else None
    if mode == 'image':
        return IMAGE_VALUES.get(text)
    raise ValueError(f'unsupported competition mode: {mode}')


def normalize_record(raw, fallback_target_id='unknown'):
    if not isinstance(raw, dict):
        return None
    recognition = raw.get('recognition') or {}
    coordinate = raw.get('coordinate') or {}
    navigation = raw.get('navigation') or {}
    try:
        record = {
            'target_id': str(raw.get('target_id') or fallback_target_id),
            'label': str(recognition.get('label', '')).strip(),
            'class_id': int(recognition.get('class_id', -1)),
            'confidence': float(recognition.get('confidence', 0.0)),
            'label_consensus': float(recognition.get('label_consensus', 0.0)),
            'latitude': float(coordinate['latitude']),
            'longitude': float(coordinate['longitude']),
            'altitude_msl_m': float(coordinate.get('altitude_msl_m', 0.0)),
            'horizontal_radius_95_m': float(
                coordinate.get('horizontal_radius_95_m', 0.0)
            ),
            'observation_count': int(raw.get('observation_count', 0)),
            'rtk_fixed': bool(raw.get('rtk_fixed', navigation.get('rtk_fixed', False))),
            'valid': bool(raw.get('valid', False)),
            'status': str(raw.get('status', '')),
            'frame_path': str(raw.get('frame_path', '')),
            'crop_path': str(raw.get('crop_path', '')),
        }
    except (KeyError, TypeError, ValueError):
        return None
    numeric = (
        record['confidence'],
        record['label_consensus'],
        record['latitude'],
        record['longitude'],
        record['altitude_msl_m'],
        record['horizontal_radius_95_m'],
    )
    if not all(math.isfinite(value) for value in numeric):
        return None
    if not (-90.0 <= record['latitude'] <= 90.0):
        return None
    if not (-180.0 <= record['longitude'] <= 180.0):
        return None
    return record


def quality_key(record):
    return (
        record['observation_count'],
        record['confidence'],
        record['label_consensus'],
        -record['horizontal_radius_95_m'],
    )


def cluster_records(records, radius_m):
    radius_m = max(0.0, radius_m)
    groups = []
    for record in sorted(records, key=quality_key, reverse=True):
        compatible = [
            group
            for group in groups
            if all(distance_m(record, member) <= radius_m for member in group)
        ]
        if not compatible:
            groups.append([record])
            continue

        # Complete-link grouping prevents A-B-C chains from joining two targets
        # whose endpoints are farther apart than the configured radius.
        group = min(
            compatible,
            key=lambda members: distance_m(record, members[0]),
        )
        group.append(record)
    return groups


def choose_competition_target(
    records,
    mode,
    required_targets=3,
    dedup_radius_m=3.0,
    allow_confirmed=False,
):
    accepted_statuses = {'finalized'}
    if allow_confirmed:
        accepted_statuses.add('confirmed')
    eligible = []
    for record in records:
        if not record:
            continue
        if (
            not record['valid']
            or record['status'] not in accepted_statuses
            or not record['rtk_fixed']
        ):
            continue
        value = mode_value(record['label'], mode)
        if value is None:
            continue
        eligible.append({**record, 'competition_value': value})

    spatial_representatives = [
        max(group, key=quality_key)
        for group in cluster_records(eligible, max(0.0, dedup_radius_m))
    ]

    # Competition rules guarantee three different non-empty labels. Treat a
    # repeated label as a duplicate track, even when coordinate scatter split
    # it into more than one spatial group.
    representatives_by_value = {}
    duplicate_labels = []
    for record in spatial_representatives:
        value = record['competition_value']
        current = representatives_by_value.get(value)
        if current is None or quality_key(record) > quality_key(current):
            if current is not None:
                duplicate_labels.append(current)
            representatives_by_value[value] = record
        else:
            duplicate_labels.append(record)

    representatives = list(representatives_by_value.values())
    representatives.sort(key=quality_key, reverse=True)
    if len(representatives) < required_targets:
        return None, representatives, duplicate_labels

    candidates = representatives[:required_targets]
    ignored = duplicate_labels + representatives[required_targets:]
    ignored.sort(key=quality_key, reverse=True)
    if mode == 'digit':
        selected = sorted(
            candidates,
            key=lambda item: (item['competition_value'], item['target_id']),
        )[required_targets // 2]
        rule = 'median_numeric_value'
    else:
        selected = max(
            candidates,
            key=lambda item: (item['competition_value'], quality_key(item)),
        )
        rule = 'highest_image_value'

    return {
        'selection_rule': rule,
        'selected': selected,
        'candidates': candidates,
    }, representatives, ignored
