from competition_selector_core import (
    choose_competition_target,
    cluster_records,
    distance_m,
    normalize_record,
)


def record(target_id, label, latitude, observations=10, confidence=0.98):
    return normalize_record({
        'target_id': target_id,
        'recognition': {
            'label': label,
            'class_id': 1,
            'confidence': confidence,
            'label_consensus': 0.95,
        },
        'coordinate': {
            'latitude': latitude,
            'longitude': 113.0,
            'altitude_msl_m': 35.0,
            'horizontal_radius_95_m': 0.3,
        },
        'observation_count': observations,
        'rtk_fixed': True,
        'valid': True,
        'status': 'confirmed',
    })


def test_digit_selects_numeric_median_and_excludes_empty():
    records = [
        record('target_001', '25', 22.0000),
        record('target_002', 'empty', 22.0001),
        record('target_003', '69', 22.0002),
        record('target_004', '52', 22.0003),
    ]
    decision, representatives, ignored = choose_competition_target(
        records, 'digit', dedup_radius_m=3.0
    )
    assert len(representatives) == 3
    assert not ignored
    assert decision['selected']['label'] == '52'
    assert decision['selected']['competition_value'] == 52


def test_digit_preserves_leading_zero_numeric_order():
    records = [
        record('target_001', '05', 22.0000),
        record('target_002', '71', 22.0002),
        record('target_003', '25', 22.0004),
    ]
    decision, _, _ = choose_competition_target(records, 'digit')
    assert decision['selected']['label'] == '25'


def test_image_selects_highest_declared_value():
    records = [
        record('target_001', '坦克', 22.0000),
        record('target_002', '卡车', 22.0002),
        record('target_003', '轰炸机', 22.0004),
        record('target_004', 'empty', 22.0006),
    ]
    decision, representatives, _ = choose_competition_target(records, 'image')
    assert len(representatives) == 3
    assert decision['selected']['label'] == '轰炸机'
    assert decision['selected']['competition_value'] == 12


def test_spatial_duplicate_uses_stronger_observation():
    records = [
        record('target_001', '25', 22.0000000, observations=5),
        record('target_009', '25', 22.0000050, observations=20),
        record('target_002', '52', 22.0002),
        record('target_003', '69', 22.0004),
    ]
    decision, representatives, _ = choose_competition_target(records, 'digit')
    assert len(representatives) == 3
    assert any(item['target_id'] == 'target_009' for item in representatives)
    assert decision['selected']['label'] == '52'


def test_duplicate_label_at_scattered_coordinates_counts_once():
    records = [
        record('target_001', '25', 22.0000, observations=5),
        record('target_009', '25', 22.0010, observations=20),
        record('target_002', '52', 22.0020),
        record('target_003', '69', 22.0030),
    ]
    decision, representatives, ignored = choose_competition_target(
        records, 'digit'
    )
    assert len(representatives) == 3
    assert len(ignored) == 1
    assert any(item['target_id'] == 'target_009' for item in representatives)
    assert decision['selected']['label'] == '52'


def test_spatial_grouping_does_not_chain_across_radius():
    metres_to_latitude = 1.0 / 111319.5
    records = [
        record('target_001', '25', 22.0, observations=30),
        record('target_002', '52', 22.0 + 2.0 * metres_to_latitude, observations=20),
        record('target_003', '69', 22.0 + 4.0 * metres_to_latitude, observations=10),
    ]
    groups = cluster_records(records, 3.0)
    assert len(groups) == 2
    assert all(
        distance_m(left, right) <= 3.0
        for group in groups
        for left in group
        for right in group
    )


def test_two_nonempty_targets_do_not_finalize():
    decision, representatives, _ = choose_competition_target(
        [
            record('target_001', '25', 22.0000),
            record('target_002', '52', 22.0002),
            record('target_003', 'empty', 22.0004),
        ],
        'digit',
    )
    assert decision is None
    assert len(representatives) == 2


if __name__ == '__main__':
    tests = sorted(
        (name, function)
        for name, function in globals().items()
        if name.startswith('test_') and callable(function)
    )
    for name, function in tests:
        function()
        print(f'PASS {name}')
    print(f'tests_passed={len(tests)}')
