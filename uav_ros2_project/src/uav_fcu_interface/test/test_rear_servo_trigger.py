"""Tests for consecutive range confirmation and one-shot servo control."""

import math

from uav_fcu_interface.rear_servo_trigger import RearServoTrigger
from uav_fcu_interface.rear_servo_trigger import RearServoTriggerConfig
from uav_fcu_interface.rear_servo_trigger import PenultimateWaypointGate


def make_trigger(**kwargs):
    """Build a trigger with the required production defaults."""
    config = RearServoTriggerConfig(
        trigger_distance_m=0.15,
        confirm_count=3,
        closed_pwm=1000,
        open_pwm=2000,
        channel=8,
        range_timeout_s=1.0,
    )
    return RearServoTrigger(config, **kwargs)


def feed(trigger, samples):
    """Feed samples at the measured 4 Hz rangefinder rate."""
    events = []
    for index, sample in enumerate(samples):
        valid = sample != 'invalid'
        distance = math.nan if not valid else sample
        event = trigger.observe(distance, valid, index * 0.25)
        events.append(event)
    return events


def test_case_1_above_threshold_does_not_trigger():
    trigger = make_trigger()
    feed(trigger, [0.20, 0.18, 0.16])
    assert not trigger.rear_servo_opened
    assert trigger.low_distance_count == 0


def test_case_2_third_consecutive_low_sample_triggers():
    trigger = make_trigger()
    events = feed(trigger, [0.14, 0.13, 0.12])
    assert [event.kind for event in events] == [
        'candidate', 'candidate', 'triggered'
    ]
    assert trigger.rear_servo_opened


def test_case_3_above_threshold_resets_consecutive_count():
    trigger = make_trigger()
    events = feed(trigger, [0.14, 0.13, 0.16, 0.14, 0.13, 0.12])
    trigger_indices = [
        index for index, event in enumerate(events)
        if event is not None and event.kind == 'triggered'
    ]
    assert trigger_indices == [5]


def test_case_4_invalid_sample_resets_consecutive_count():
    trigger = make_trigger()
    feed(trigger, [0.14, 'invalid', 0.13, 0.12])
    assert not trigger.rear_servo_opened
    assert trigger.low_distance_count == 2


def test_case_5_opened_state_is_latched():
    sent_commands = []

    def sender(channel, pwm):
        sent_commands.append((channel, pwm))
        return True

    trigger = make_trigger(
        dry_run=False,
        real_control_enabled=True,
        command_sender=sender,
    )
    feed(trigger, [0.14, 0.13, 0.12, 0.30, 0.40, 0.10])
    assert trigger.rear_servo_opened
    assert trigger.low_distance_count == 3
    assert sent_commands == [(8, 2000)]


def test_case_6_dry_run_never_calls_command_sender():
    sent_commands = []

    def sender(channel, pwm):
        sent_commands.append((channel, pwm))
        return True

    trigger = make_trigger(
        dry_run=True,
        real_control_enabled=True,
        command_sender=sender,
    )
    events = feed(trigger, [0.14, 0.13, 0.12])
    assert events[-1].action == 'dry_run'
    assert trigger.rear_servo_opened
    assert sent_commands == []


def test_timeout_resets_unfinished_confirmation():
    trigger = make_trigger()
    trigger.observe(0.14, True, 0.0)
    trigger.observe(0.13, True, 0.25)
    assert trigger.check_timeout(1.26)
    assert trigger.low_distance_count == 0


def test_waypoint_gate_opens_at_original_mission_penultimate():
    gate = PenultimateWaypointGate()
    assert gate.observe_mission(('original',), 8, 2) == (
        'mission_changed_closed'
    )
    assert gate.target_seq == 6
    assert not gate.observe_reached(5)
    assert gate.observe_reached(6)
    assert gate.is_open


def test_waypoint_gate_recomputes_after_attack_mission_upload():
    gate = PenultimateWaypointGate()
    gate.observe_mission(('original',), 8, 2)
    assert gate.target_seq == 6

    assert gate.observe_mission(('attack',), 13, 2) == (
        'mission_changed_closed'
    )
    assert gate.target_seq == 11
    assert not gate.observe_reached(6)
    assert not gate.is_open
    assert gate.observe_reached(11)
    assert gate.is_open


def test_waypoint_gate_resets_for_a_new_mission_revision():
    gate = PenultimateWaypointGate()
    gate.observe_mission(('first',), 8, 2)
    gate.observe_reached(6)
    assert gate.is_open

    gate.observe_mission(('second',), 10, 3)
    assert gate.target_seq == 8
    assert not gate.is_open


def test_waypoint_gate_recovers_when_node_starts_after_penultimate():
    gate = PenultimateWaypointGate()
    event = gate.observe_mission(('mission',), 8, 7)
    assert event == 'mission_changed_open'
    assert gate.is_open


def test_waypoint_gate_fails_closed_for_short_or_missing_mission():
    gate = PenultimateWaypointGate()
    gate.observe_mission((), 0, 0)
    assert gate.target_seq is None
    assert not gate.is_open
    assert not gate.observe_reached(0)


def test_low_range_cannot_trigger_until_penultimate_is_reached():
    gate = PenultimateWaypointGate()
    trigger = make_trigger()
    gate.observe_mission(('attack',), 13, 2)

    for index, distance in enumerate((0.10, 0.09, 0.08)):
        trigger.observe(distance, gate.is_open, index * 0.25)
    assert not trigger.rear_servo_opened
    assert trigger.low_distance_count == 0

    assert gate.observe_reached(11)
    events = []
    for index, distance in enumerate((0.10, 0.09, 0.08), start=3):
        events.append(
            trigger.observe(distance, gate.is_open, index * 0.25)
        )
    assert [event.kind for event in events] == [
        'candidate', 'candidate', 'triggered'
    ]
