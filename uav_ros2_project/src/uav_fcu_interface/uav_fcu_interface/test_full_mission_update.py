"""Full replacement, reconciliation, progress and one-attempt regressions."""
import asyncio
import copy
import time
from types import SimpleNamespace

import pytest

from uav_fcu_interface.test_abcdr_mission import prepare_dynamic_update_node


def scenario(actual='DYNAMIC', push_error=False, after=6, reached=4):
    node, points, _, events = prepare_dynamic_update_node()
    node.release_command_enabled = True
    original = [node.make_nav_waypoint(22 + i * .001, 113, 35, 15) for i in range(14)]
    mission, _ = node.build_composite_mission(points, original, 5, 9)
    node._safe_composite_mission = tuple(copy.deepcopy(mission))
    node.composite_expected_waypoints = copy.deepcopy(mission)
    node.current_waypoints = SimpleNamespace(current_seq=5, waypoints=mission)
    node.last_reached_seq = reached
    calls = []

    async def push(waypoints, **kwargs):
        assert kwargs['retry'] is False
        assert len(waypoints) == len(mission)
        calls.append(('push', copy.deepcopy(waypoints)))
        if push_error:
            raise RuntimeError('injected push error')

    async def pull(**kwargs):
        calls.append(('pull', None))
        if actual == 'UNKNOWN':
            raise RuntimeError('injected pull error')
        observed = copy.deepcopy(mission if actual == 'SAFE' else calls[0][1])
        if actual == 'INCONSISTENT':
            observed[8].param2 += 100
        node.current_waypoints = SimpleNamespace(current_seq=after, waypoints=observed)
        return node.current_waypoints

    async def set_current(seq, **kwargs):
        calls.append(('set_current', seq))
        node.current_waypoints.current_seq = seq

    node.push_mission_async = push
    node.pull_mission_async = pull
    node.set_current_mission_item_async = set_current
    return node, events, calls


def run(node):
    node._dynamic_r_worker({'timestamp_monotonic': time.monotonic(), 'current_seq': 5})


def test_full_success_only_r_changes_and_safe_cache_cannot_be_mutated():
    node, events, calls = scenario()
    safe = node.safe_composite_mission
    run(node)
    assert [c[0] for c in calls] == ['push', 'pull']
    dynamic = calls[0][1]
    assert len(safe) == len(dynamic)
    fields = ('frame', 'command', 'param1', 'param2', 'param3', 'param4',
              'x_lat', 'y_long', 'z_alt', 'autocontinue', 'is_current')
    changed = [i for i, (a, b) in enumerate(zip(safe, dynamic))
               if any(getattr(a, f) != getattr(b, f) for f in fields)]
    assert changed == [7]
    assert dynamic[8].command == 183
    assert dynamic[8].param1 == node.servo_channel
    assert dynamic[8].param2 == node.release_pwm
    assert node._dynamic_update_verified
    safe[7].x_lat += 1
    assert node.safe_composite_mission[7].x_lat != safe[7].x_lat
    result = next(v for k, v in events if k == 'dynamic_mission_verified')
    for field in ('dynamic_trigger_timestamp', 'r_calc_start', 'r_calc_done',
                  'second_full_push_start', 'second_full_push_done',
                  'second_full_push_duration_ms', 'second_full_pull_start',
                  'second_full_pull_done', 'second_full_pull_duration_ms',
                  'second_full_verify_start', 'second_full_verify_done',
                  'verify_cpu_duration_ms', 'dynamic_full_update_total_ms'):
        assert result[field] >= 0
    assert result['current_seq_before_update'] == 5
    assert result['current_seq_after_update'] == 6
    assert result['r_source'] == 'DYNAMIC'
    names = [k for k, _ in events]
    sequence = ['second_full_push_start', 'second_full_push_done',
                'second_full_pull_start', 'second_full_pull_done',
                'second_full_verify_start', 'second_full_verify_pass',
                'dynamic_mission_verified']
    assert [names.index(n) for n in sequence] == sorted(names.index(n) for n in sequence)


@pytest.mark.parametrize('field', ['frame', 'command', 'param1', 'param2', 'param3',
                                   'param4', 'x_lat', 'y_long', 'z_alt', 'autocontinue'])
def test_structure_mismatch_refuses_upload(field):
    node, events, calls = scenario()
    build = node.build_dynamic_mission
    def corrupt(candidate):
        mission = build(candidate)
        setattr(mission[8], field, getattr(mission[8], field) + 1)
        return mission
    node.build_dynamic_mission = corrupt
    run(node)
    assert calls == []
    assert 'dynamic_mission_structure_mismatch' in [k for k, _ in events]


def test_calculation_failure_leaves_safe_without_any_service_call():
    node, events, calls = scenario()
    node.compute_dynamic_r = lambda *_: {'valid': False, 'reason': 'injected'}
    run(node)
    assert calls == []
    assert any(k == 'dynamic_update_skipped' and v['reason'] == 'r_calculation_failed'
               and v['r_source'] == 'SAFE' for k, v in events)


def test_count_mismatch_and_unverified_cache_refuse_second_upload():
    node, events, calls = scenario()
    build = node.build_dynamic_mission
    node.build_dynamic_mission = lambda candidate: build(candidate)[:-1]
    run(node)
    assert calls == []
    assert any(k == 'dynamic_mission_structure_mismatch' for k, _ in events)
    node, events, calls = scenario()
    node._safe_composite_mission = None
    run(node)
    assert calls == []
    assert any(k == 'dynamic_mission_failed' for k, _ in events)


def test_safe_cache_is_not_saved_when_first_full_verification_fails():
    node, _, _ = scenario()
    node._safe_composite_mission = None
    node.insert_wp_index = 5
    node.resume_wp_index = 8
    node.clear_mission_before_full_push = False
    node.current_mode = lambda: 'AUTO'
    pushed = []
    pulls = []
    original = list(node.safe_composite_mission or node.composite_expected_waypoints)
    async def push(waypoints):
        pushed.extend(copy.deepcopy(waypoints))
    async def pull():
        pulls.append(True)
        if len(pulls) == 1:
            return SimpleNamespace(current_seq=2, waypoints=original)
        observed = copy.deepcopy(pushed)
        observed[7].x_lat += .01
        return SimpleNamespace(current_seq=2, waypoints=observed)
    node.push_mission_async = push
    node.pull_mission_async = pull
    with pytest.raises(RuntimeError, match='verification failed'):
        asyncio.run(node.handle_goto_global_composite_async(node.composite_route_points))
    assert node.safe_composite_mission is None


@pytest.mark.parametrize('deadline', ['current', 'reached'])
def test_deadline_before_upload(deadline):
    node, events, calls = scenario()
    if deadline == 'current':
        node.current_waypoints.current_seq = 7
    else:
        node.last_reached_seq = 7
    run(node)
    assert calls == []
    assert any(k == 'dynamic_update_too_late' and v['r_source'] == 'SAFE' for k, v in events)


@pytest.mark.parametrize('actual', ['SAFE', 'DYNAMIC', 'INCONSISTENT', 'UNKNOWN'])
def test_push_error_reconciles_once_without_retry(actual):
    node, events, calls = scenario(actual=actual, push_error=True)
    run(node)
    run(node)
    assert [k for k, _ in calls] == ['push', 'pull']
    assert any(k == 'second_full_push_failed' for k, _ in events)
    if actual == 'DYNAMIC':
        assert node._dynamic_update_verified
    elif actual == 'SAFE':
        assert not node._dynamic_update_verified
        assert any(v.get('mission_state') == 'SAFE' and v.get('r_source') == 'SAFE'
                   for _, v in events)
    else:
        assert not node._dynamic_update_verified
        assert any(k == 'dynamic_mission_failed' and v['mission_state'] == actual
                   for k, v in events)


@pytest.mark.parametrize('after', [5, 6])
def test_natural_progress_never_set_current(after):
    node, _, calls = scenario(after=after)
    run(node)
    assert node._dynamic_update_verified
    assert [k for k, _ in calls] == ['push', 'pull']


@pytest.mark.parametrize('reached,resume', [(4, 5), (5, 6)])
def test_reset_recovers_from_reached_evidence(reached, resume):
    node, events, calls = scenario(after=0, reached=reached)
    run(node)
    assert calls[-1] == ('set_current', resume)
    assert reached < resume < 7
    assert node._dynamic_update_verified
    assert any(k == 'mission_progress_recovery' for k, _ in events)


@pytest.mark.parametrize('after,reached', [(0, -1), (0, 6), (7, 5)])
def test_unknown_or_late_progress_never_guesses(after, reached):
    node, events, calls = scenario(after=after, reached=reached)
    run(node)
    assert [k for k, _ in calls] == ['push', 'pull']
    assert not node._dynamic_update_verified
    assert any(k == 'mission_progress_unknown' for k, _ in events)


@pytest.mark.parametrize('reason', ['r_reached', 'd_reached', 'mission_complete',
                                    'task_cleared', 'dynamic_update_too_late'])
def test_cancellation_during_push_allows_pull_but_never_retry(reason):
    node, _, calls = scenario(push_error=True)
    push = node.push_mission_async
    async def cancelled(*args, **kwargs):
        node._cancel_dynamic_update(reason)
        return await push(*args, **kwargs)
    node.push_mission_async = cancelled
    run(node)
    run(node)
    assert [k for k, _ in calls] == ['push', 'pull']
    assert not node._dynamic_update_verified


def test_wire_request_is_full_and_failed_service_is_not_retried():
    from uav_fcu_interface.fcu_interface_mavros_node import FcuInterfaceMavrosNode
    node, _, _ = scenario()
    node.mission_push_client = object()
    requests = []
    async def service(client, request, timeout, name, **kwargs):
        requests.append(request)
        return SimpleNamespace(success=False, wp_transfered=0)
    node.call_service_async = service
    with pytest.raises(RuntimeError, match='rejected'):
        asyncio.run(FcuInterfaceMavrosNode.push_mission_async(
            node, list(node.safe_composite_mission), retry=False))
    assert len(requests) == 1
    assert requests[0].start_index == 0
    assert len(requests[0].waypoints) == len(node.safe_composite_mission)


@pytest.mark.parametrize('late', [False, True])
def test_recovery_rechecks_latest_progress_at_service_dispatch(late):
    from uav_fcu_interface.fcu_interface_mavros_node import FcuInterfaceMavrosNode
    node, _, calls = scenario()
    node.current_waypoints.current_seq = 0
    node.service_availability_timeout_sec = .01
    requests = []
    class Client:
        def wait_for_service(self, **kwargs):
            node.current_waypoints.current_seq = 7 if late else 6
            return True
        def call_async(self, request):
            requests.append(request)
    node.mission_set_current_client = Client()
    if late:
        with pytest.raises(RuntimeError, match='mission_current_r'):
            asyncio.run(FcuInterfaceMavrosNode.set_current_mission_item_async(
                node, 5, dynamic_started=time.monotonic()))
    else:
        asyncio.run(FcuInterfaceMavrosNode.set_current_mission_item_async(
            node, 5, dynamic_started=time.monotonic()))
    assert requests == []


def test_manager_fault_routes_only_current_executing_task_to_existing_fail():
    import json
    import threading
    from uav_mission_manager.mission_manager_node import MissionManagerNode, MissionState
    calls = []
    node = SimpleNamespace(_lock=threading.RLock(), state=MissionState.EXECUTING,
                           active_target={'id': 'task1'}, fail=calls.append)
    for event in ({'event': 'dynamic_mission_failed', 'task_id': 'old'},
                  {'event': 'second_full_push_failed', 'task_id': 'task1'},
                  {'event': 'dynamic_mission_failed', 'task_id': 'task1',
                   'reason': 'mission_inconsistent'}):
        MissionManagerNode.dynamic_mission_failure_callback(
            node, SimpleNamespace(data=json.dumps(event)))
    assert calls == ['mission_inconsistent']


def test_summary_reports_full_timing_and_reconciled_push_failure():
    from uav_bringup.flight_summary_logger_node import FlightSummaryLoggerNode
    node, events, _ = scenario(push_error=True)
    run(node)
    record = next(v for k, v in events if k == 'dynamic_mission_verified')
    logger = FlightSummaryLoggerNode.__new__(FlightSummaryLoggerNode)
    text = logger._format_human_event({'event': 'dynamic_mission_verified', **record})
    assert 'SECOND FULL MISSION UPDATE' in text
    assert 'push: FAILED (reconciled)' in text
    assert 'before: seq5' in text and 'after: seq6' in text
    assert 'result: VERIFIED' in text and 'source: DYNAMIC' in text


def test_push_deadline_is_rechecked_after_waiting_for_service():
    from uav_fcu_interface.fcu_interface_mavros_node import FcuInterfaceMavrosNode
    node, _, _ = scenario()
    node.service_availability_timeout_sec = .01
    requests = []
    class Client:
        def wait_for_service(self, **kwargs):
            node.last_reached_seq = 7
            return True
        def call_async(self, request):
            requests.append(request)
    node.mission_push_client = Client()
    with pytest.raises(RuntimeError, match='r_reached'):
        asyncio.run(FcuInterfaceMavrosNode.push_mission_async(
            node, list(node.safe_composite_mission), retry=False,
            started_monotonic=time.monotonic()))
    assert requests == []


@pytest.mark.parametrize('before,after,reached_after', [(6, 6, 6), (6, 6, 5), (5, 6, 6)])
def test_natural_progress_accepts_u_reached_during_second_full_pull(
        before, after, reached_after):
    import threading
    node, events, calls = scenario(after=after, reached=5)
    node.current_waypoints.current_seq = before
    node._dynamic_update_started = True
    node.log_state = 'EXECUTING'
    pull = node.pull_mission_async
    callback_errors = []

    async def reached_during_pull(**kwargs):
        # Reproduce the supplied SITL ordering on a real callback thread.
        def callback():
            try:
                node.waypoint_reached_callback(SimpleNamespace(wp_seq=reached_after))
            except Exception as error:
                callback_errors.append(error)
        thread = threading.Thread(target=callback)
        thread.start()
        thread.join(timeout=1)
        assert not thread.is_alive()
        assert callback_errors == []
        return await pull(**kwargs)

    node.pull_mission_async = reached_during_pull
    run(node)
    assert [name for name, _ in calls] == ['push', 'pull']
    names = [name for name, _ in events]
    assert not {'mission_progress_recovery', 'mission_progress_unknown',
                'dynamic_mission_failed', 'dynamic_update_cancelled'} & set(names)
    result = next(v for k, v in events if k == 'dynamic_mission_verified')
    assert result['r_source'] == 'DYNAMIC'
    assert result['current_seq_before_update'] == before
    assert result['last_reached_seq_before_update'] == 5
    assert result['current_seq_after_update'] == after
    assert result['last_reached_seq_after_update'] == reached_after
    assert any(k == 'mission_progress_safe'
               and v['reason'] == 'r_still_future_natural_progress' for k, v in events)
    assert any(k == 'mission_progress_accepted' and v['action'] == 'no_set_current'
               for k, v in events)
    node.waypoints_callback(SimpleNamespace(current_seq=7,
                                            waypoints=node.current_waypoints.waypoints))
    names = [k for k, _ in events]
    order = ['second_full_verify_pass', 'mission_progress_check', 'mission_progress_safe',
             'mission_progress_accepted', 'dynamic_mission_verified', 'mission_current_r']
    assert [names.index(k) for k in order] == sorted(names.index(k) for k in order)


@pytest.mark.parametrize('current,reached', [(7, 6), (7, 7), (8, 7), (6, 7)])
def test_r_deadline_during_second_pull_remains_too_late(current, reached):
    node, events, calls = scenario(after=current, reached=5)
    node.current_waypoints.current_seq = 6
    pull = node.pull_mission_async
    async def advance(**kwargs):
        with node._dynamic_state_lock:
            node.last_reached_seq = reached
        return await pull(**kwargs)
    node.pull_mission_async = advance
    run(node)
    assert [k for k, _ in calls] == ['push', 'pull']
    assert node._dynamic_update_result == 'UPDATE_TOO_LATE'
    assert any(k == 'dynamic_update_too_late' for k, _ in events)
    assert not node._dynamic_update_verified
    assert not any(k == 'dynamic_mission_verified' for k, _ in events)


def test_actual_current_regression_still_requires_recovery():
    node, events, calls = scenario(after=2, reached=5)
    node.current_waypoints.current_seq = 6
    run(node)
    assert calls[-1] == ('set_current', 6)
    assert any(k == 'mission_progress_recovery' for k, _ in events)
    assert not any(k == 'mission_progress_accepted' for k, _ in events)


@pytest.mark.parametrize('reached_after_check,verified', [(6, True), (7, False)])
def test_reached_between_progress_check_and_commit_is_rechecked(reached_after_check, verified):
    node, events, _ = scenario(after=6, reached=5)
    node.current_waypoints.current_seq = 6
    commit = node._commit_dynamic_r_verified
    def reached_then_commit(*args, **kwargs):
        with node._dynamic_state_lock:
            node.last_reached_seq = reached_after_check
        return commit(*args, **kwargs)
    node._commit_dynamic_r_verified = reached_then_commit
    run(node)
    assert node._dynamic_update_verified is verified
    assert any(k == 'dynamic_mission_verified' for k, _ in events) is verified


def test_progress_snapshot_and_deadline_use_shared_state_lock():
    node, events, _ = scenario(after=6, reached=6)
    node.current_waypoints.current_seq = 6
    node._dynamic_metrics.update(current_seq_before_update=6,
                                last_reached_seq_before_update=5)
    current = node._current_mission_seq
    reads = []
    def locked_read():
        assert node._dynamic_state_lock.locked()
        reads.append(True)
        return current()
    node._current_mission_seq = locked_read
    assert asyncio.run(node._check_dynamic_progress(time.monotonic()))
    assert reads
    check = next(v for k, v in events if k == 'mission_progress_check')
    assert check['current_seq_after_update'] == check['last_reached_seq_after_update'] == 6
    assert 'dynamic_update_state' in check


def test_progress_logs_include_before_after_and_no_set_current():
    from uav_bringup.flight_summary_logger_node import FlightSummaryLoggerNode
    logger = FlightSummaryLoggerNode.__new__(FlightSummaryLoggerNode)
    record = dict(event='mission_progress_check', current_seq_before_update=6,
                  last_reached_seq_before_update=5, current_seq_after_update=6,
                  last_reached_seq_after_update=6, r_seq=7)
    text = logger._format_human_event(record)
    for field, value in record.items():
        if field != 'event':
            assert f'{field}: {value}' in text
    assert 'action: no_set_current' in logger._format_human_event(
        dict(event='mission_progress_accepted', action='no_set_current'))
