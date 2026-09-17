from competition_selector_core import (
    choose_competition_target,
    cluster_records,
    distance_m,
    normalize_record,
    suppress_nearby_records,
)


def record(
    target_id,
    label,
    latitude,
    observations=10,
    confidence=0.98,
    status='finalized',
):
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
        'status': status,
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


def test_finalized_packet_results_are_eligible():
    decision, representatives, ignored = choose_competition_target(
        [
            record('target_001', '25', 22.0000, status='finalized'),
            record('target_002', '52', 22.0002, status='finalized'),
            record('target_003', '69', 22.0004, status='finalized'),
        ],
        'digit',
    )
    assert not ignored
    assert len(representatives) == 3
    assert decision['selected']['label'] == '52'


def test_confirmed_results_cannot_trigger_competition_selection():
    decision, representatives, ignored = choose_competition_target(
        [
            record('target_001', '25', 22.0000, status='confirmed'),
            record('target_002', '52', 22.0002, status='confirmed'),
            record('target_003', '69', 22.0004, status='confirmed'),
        ],
        'digit',
    )
    assert decision is None
    assert not representatives
    assert not ignored


def test_static_legacy_mode_can_explicitly_allow_confirmed_results():
    decision, representatives, ignored = choose_competition_target(
        [
            record('target_001', '25', 22.0000, status='confirmed'),
            record('target_002', '52', 22.0002, status='confirmed'),
            record('target_003', '69', 22.0004, status='confirmed'),
        ],
        'digit',
        allow_confirmed=True,
    )
    assert not ignored
    assert len(representatives) == 3
    assert decision['selected']['label'] == '52'


def test_same_physical_target_with_label_flicker_counts_once():
    metres_to_latitude = 1.0 / 111319.5
    decision, representatives, ignored = choose_competition_target(
        [
            record('climb_packet', '25', 22.0, observations=36),
            record(
                'level_packet',
                '52',
                22.0 + 7.0 * metres_to_latitude,
                observations=55,
            ),
            record('target_b', '69', 22.0 + 20.0 * metres_to_latitude),
            record('target_c', '71', 22.0 + 40.0 * metres_to_latitude),
        ],
        'digit',
    )
    assert decision is not None
    assert len(representatives) == 3
    assert any(item['target_id'] == 'level_packet' for item in representatives)
    assert not any(item['target_id'] == 'climb_packet' for item in representatives)
    assert ignored[0]['_selection_reason'] == 'within_distinct_target_distance'
    assert ignored[0]['_suppressed_by_target_id'] == 'level_packet'


def test_nearby_suppression_keeps_representatives_pairwise_over_ten_metres():
    metres_to_latitude = 1.0 / 111319.5
    records = [
        record('strong_a', '25', 22.0, observations=50),
        record('weak_chain', '52', 22.0 + 7.0 * metres_to_latitude, observations=20),
        record('target_b', '69', 22.0 + 14.0 * metres_to_latitude, observations=30),
        record('target_c', '71', 22.0 + 30.0 * metres_to_latitude, observations=25),
    ]
    representatives, suppressed = suppress_nearby_records(records, 10.0)
    assert len(representatives) == 3
    assert [item['target_id'] for item in suppressed] == ['weak_chain']
    assert all(
        distance_m(left, right) > 10.0
        for index, left in enumerate(representatives)
        for right in representatives[index + 1:]
    )


def test_two_targets_plus_nearby_misclassification_cannot_finalize():
    metres_to_latitude = 1.0 / 111319.5
    decision, representatives, ignored = choose_competition_target(
        [
            record('target_a_best', '25', 22.0, observations=40),
            record(
                'target_a_wrong_label',
                '52',
                22.0 + 9.0 * metres_to_latitude,
                observations=12,
            ),
            record('target_b', '69', 22.0 + 25.0 * metres_to_latitude),
        ],
        'digit',
    )
    assert decision is None
    assert len(representatives) == 2
    assert len(ignored) == 1
    assert ignored[0]['target_id'] == 'target_a_wrong_label'


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
