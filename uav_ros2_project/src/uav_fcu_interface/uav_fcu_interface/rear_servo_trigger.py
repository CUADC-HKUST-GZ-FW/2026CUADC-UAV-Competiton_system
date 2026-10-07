"""Pure state machine for one-shot, range-triggered rear-servo control."""

from dataclasses import dataclass
import math


class PenultimateWaypointGate:
    """Open once the current FCU mission reaches its penultimate item."""

    def __init__(self):
        self.mission_signature = None
        self.mission_count = 0
        self.target_seq = None
        self.is_open = False

    def observe_mission(self, signature, mission_count, current_seq):
        """Track a mission revision and return a gate state-change label."""
        signature = tuple(signature)
        mission_count = int(mission_count)
        current_seq = int(current_seq)
        changed = signature != self.mission_signature

        if changed:
            self.mission_signature = signature
            self.mission_count = mission_count
            self.target_seq = (
                mission_count - 2 if mission_count >= 2 else None
            )
            self.is_open = False

        if (
            not self.is_open
            and self.target_seq is not None
            and current_seq > self.target_seq
        ):
            self.is_open = True
            return 'mission_changed_open' if changed else 'opened_by_progress'

        if changed:
            return 'mission_changed_closed'
        return None

    def observe_reached(self, reached_seq):
        """Open when MAVROS reports the penultimate item reached or passed."""
        if self.target_seq is None or self.is_open:
            return False
        if int(reached_seq) != self.target_seq:
            return False
        self.is_open = True
        return True


@dataclass(frozen=True)
class RearServoTriggerConfig:
    """Configuration for the rear-servo trigger state machine."""

    trigger_distance_m: float = 0.15
    confirm_count: int = 3
    closed_pwm: int = 1000
    open_pwm: int = 2000
    channel: int = 8
    range_timeout_s: float = 1.0

    def validate(self):
        """Raise ValueError when a safety-critical setting is invalid."""
        if not math.isfinite(self.trigger_distance_m):
            raise ValueError('trigger_distance_m must be finite')
        if self.trigger_distance_m < 0.0:
            raise ValueError('trigger_distance_m must be >= 0')
        if self.confirm_count < 1:
            raise ValueError('confirm_count must be >= 1')
        if not 800 <= self.closed_pwm <= 2200:
            raise ValueError('closed_pwm must be within [800, 2200]')
        if not 800 <= self.open_pwm <= 2200:
            raise ValueError('open_pwm must be within [800, 2200]')
        if self.closed_pwm == self.open_pwm:
            raise ValueError('closed_pwm and open_pwm must differ')
        if not 1 <= self.channel <= 16:
            raise ValueError('channel must be within [1, 16]')
        if self.range_timeout_s <= 0.0:
            raise ValueError('range_timeout_s must be > 0')


@dataclass(frozen=True)
class RearServoTriggerEvent:
    """State-machine transition emitted after one observation."""

    kind: str
    distance_m: float
    confirm_count: int
    action: str = 'none'


class RearServoTrigger:
    """Require consecutive low readings, then latch OPENED for this run."""

    def __init__(
        self,
        config,
        *,
        dry_run=True,
        real_control_enabled=False,
        command_sender=None,
    ):
        config.validate()
        self.config = config
        self.dry_run = bool(dry_run)
        self.real_control_enabled = bool(real_control_enabled)
        self.command_sender = command_sender
        self.low_distance_count = 0
        self.rear_servo_opened = False
        self.last_sample_time = None

    def observe(self, distance_m, valid, now):
        """Consume one sample and return a transition event, if any."""
        now = float(now)
        distance_m = float(distance_m)

        if self.rear_servo_opened:
            return None

        if (
            self.last_sample_time is not None
            and now - self.last_sample_time > self.config.range_timeout_s
        ):
            self.low_distance_count = 0
        self.last_sample_time = now

        if not valid or not math.isfinite(distance_m):
            previous_count = self.low_distance_count
            self.low_distance_count = 0
            if previous_count:
                return RearServoTriggerEvent(
                    'reset_invalid', distance_m, 0
                )
            return None

        if distance_m > self.config.trigger_distance_m:
            previous_count = self.low_distance_count
            self.low_distance_count = 0
            if previous_count:
                return RearServoTriggerEvent(
                    'reset_above_threshold', distance_m, 0
                )
            return None

        self.low_distance_count += 1
        if self.low_distance_count < self.config.confirm_count:
            return RearServoTriggerEvent(
                'candidate', distance_m, self.low_distance_count
            )

        self.rear_servo_opened = True
        action = self._execute_open_command()
        return RearServoTriggerEvent(
            'triggered', distance_m, self.low_distance_count, action
        )

    def check_timeout(self, now):
        """Reset an unfinished candidate after range data becomes stale."""
        if self.rear_servo_opened or self.last_sample_time is None:
            return False
        if float(now) - self.last_sample_time <= self.config.range_timeout_s:
            return False
        was_counting = self.low_distance_count > 0
        self.low_distance_count = 0
        return was_counting

    def _execute_open_command(self):
        if self.dry_run:
            return 'dry_run'
        if not self.real_control_enabled:
            return 'blocked'
        if self.command_sender is None:
            return 'blocked'
        try:
            dispatched = self.command_sender(
                self.config.channel,
                self.config.open_pwm,
            )
        except Exception:
            return 'command_failed'
        return 'command_sent' if dispatched else 'command_failed'
