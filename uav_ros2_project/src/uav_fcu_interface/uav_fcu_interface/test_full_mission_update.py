"""Second full-mission dynamic-R update safety regressions."""

import copy
import time
from types import SimpleNamespace

import pytest

from uav_fcu_interface.test_abcdr_mission import prepare_dynamic_update_node


def _scenario(actual='DYNAMIC', push_error=False, cancel_reason=None):
    node, points, mission, events = prepare_dynamic_update_node()
    node.current_waypoints.current_seq = node.composite_seq_u
    node.last_reached_seq = node.composite_seq_a
    calls = []
    fcu_mission = copy.deepcopy(mission)

    async def push(waypoints, retry=True, started_monotonic=None):
        nonlocal fcu_mission
        calls.append(('push', copy.deepcopy(waypoints), retry, started_monotonic))
        fcu_mission = copy.deepcopy(waypoints)
        if actual == 'SAFE':
            fcu_mission = copy.deepcopy(mission)
        elif actual == 'INCONSISTENT':
            fcu_mission[node.composite_seq_d].x_lat += 0.01
        if cancel_reason:
            node._cancel_dynamic_update(cancel_reason)
        if push_error:
            raise RuntimeError('injected second full push failure')
        return SimpleNamespace(success=True, wp_transfered=len(waypoints))

    async def pull(_started, reconcile=False):
        calls.append(('pull', reconcile))
        if actual == 'UNKNOWN':
            raise RuntimeError('injected pull failure')
        return SimpleNamespace(
            current_seq=node.composite_seq_u,
            waypoints=copy.deepcopy(fcu_mission),
        )

    node.push_mission_async = push
    node._pull_dynamic_mission_async = pull
    return node, points, mission, events, calls


def _run(node):
    node._dynamic_r_worker({
        'timestamp_monotonic': time.monotonic(),
        'current_seq': node.composite_seq_u,
    })


def test_second_full_upload_replaces_complete_mission_and_changes_only_r():
    node, _points, mission, events, calls = _scenario()
    safe = node.safe_composite_mission

    _run(node)

    pushes = [call for call in calls if call[0] == 'push']
    assert len(pushes) == 1
    uploaded = pushes[0][1]
    assert pushes[0][2] is False  # no retry for the in-flight second upload
    assert len(uploaded) == len(mission) == 14
    changed = []
    for seq, (before, after) in enumerate(zip(mission, uploaded)):
        if tuple(getattr(before, f) for f in ('frame','command','param1','param2','param3','param4','x_lat','y_long','z_alt','autocontinue','is_current')) != tuple(getattr(after, f) for f in ('frame','command','param1','param2','param3','param4','x_lat','y_long','z_alt','autocontinue','is_current')):
            changed.append(seq)
    assert changed == [node.composite_seq_r] == [7]
    assert uploaded[node.composite_seq_release].command == 183
    assert node.composite_seq_release == 8
    assert node.composite_seq_d == 9
    assert node._dynamic_update_verified
    names = [name for name, _fields in events]
    assert names.index('second_full_push_start') < names.index('second_full_push_done')
    assert names.index('second_full_push_done') < names.index('second_full_push_ack_confirmed')
    assert 'second_full_pull_start' not in names
    assert 'second_full_pull_done' not in names
    assert 'second_full_verify_start' not in names
    assert 'second_full_verify_pass' not in names
    assert not [call for call in calls if call[0] == 'pull']
    assert names.index('mission_progress_accepted') < names.index('dynamic_mission_verified')
    safe[7].x_lat += 1.0
    assert node.safe_composite_mission[7].x_lat != safe[7].x_lat


def test_successful_second_full_push_does_not_pull_back():
    node, _points, _mission, events, calls = _scenario()

    _run(node)

    assert [call for call in calls if call[0] == 'pull'] == []
    names = [name for name, _fields in events]
    assert 'second_full_push_ack_confirmed' in names
    assert 'dynamic_mission_verified' in names
    verified = [fields for name, fields in events
                if name == 'dynamic_mission_verified'][-1]
    assert verified['verify_duration_ms'] == 0.0
    assert 'push_ack_full_transfer=true' in verified['verification_reason']


@pytest.mark.parametrize(
    'field',
    ('frame', 'command', 'param1', 'param2', 'param3', 'param4',
     'x_lat', 'y_long', 'z_alt', 'autocontinue', 'is_current'),
)
def test_structure_guard_rejects_non_r_changes(field):
    node, points, _mission, _events, calls = _scenario()
    candidate = node.compute_dynamic_r({'ground_speed_mps': 20.0}, points['C'])
    expected = node.build_dynamic_mission(candidate)
    seq = node.composite_seq_d
    value = getattr(expected[seq], field)
    setattr(expected[seq], field, (not value) if isinstance(value, bool) else value + 1)

    assert not node.dynamic_mission_structure_matches(expected)
    assert calls == []


@pytest.mark.parametrize(
    'actual,expected_result,verified',
    (
        ('SAFE', 'R_SAFE_FALLBACK', False),
        ('DYNAMIC', 'DYNAMIC_R', True),
        ('INCONSISTENT', 'UPDATE_FAILED', False),
        ('UNKNOWN', 'UPDATE_FAILED', False),
    ),
)
def test_failed_second_full_push_reconciles_without_compensating_write(
        actual, expected_result, verified):
    node, _points, _mission, events, calls = _scenario(
        actual=actual, push_error=True
    )

    _run(node)

    pushes = [call for call in calls if call[0] == 'push']
    pulls = [call for call in calls if call[0] == 'pull']
    assert len(pushes) == 1
    assert len(pulls) == 1
    assert pulls[0][1] is True  # read-only reconciliation
    assert node._dynamic_update_result == expected_result
    assert node._dynamic_update_verified is verified
    assert not any('restore' in name for name, _fields in events)


@pytest.mark.parametrize(
    'reason',
    ('r_reached', 'd_reached', 'composite_complete', 'mission_complete',
     'task_cleared'),
)
def test_terminal_cancellation_prevents_new_second_full_upload(reason):
    node, _points, _mission, events, calls = _scenario()
    node._cancel_dynamic_update(reason)

    _run(node)
    _run(node)

    assert [call for call in calls if call[0] == 'push'] == []
    assert node._dynamic_update_cancelled
    assert node._dynamic_worker_cancel_reason == reason
    assert 'dynamic_mission_verified' not in [name for name, _fields in events]


def test_failed_attempt_is_latched_and_not_retried():
    node, _points, _mission, _events, calls = _scenario(
        actual='SAFE', push_error=True
    )

    _run(node)
    first_count = len([call for call in calls if call[0] == 'push'])
    _run(node)

    assert first_count == 1
    assert len([call for call in calls if call[0] == 'push']) == first_count
    assert node._second_full_push_attempted
