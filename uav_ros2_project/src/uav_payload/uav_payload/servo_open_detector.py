"""Pure state machine for detecting repeatable servo-open transitions."""

from dataclasses import dataclass


class ServoOpenState:
    """States used by the standalone servo-open detector."""

    WAITING_FOR_CLOSED = 'WAITING_FOR_CLOSED'
    ARMED = 'ARMED'
    CANDIDATE = 'CANDIDATE'
    WAITING_FOR_CLOSE = 'WAITING_FOR_CLOSE'


@dataclass(frozen=True)
class ServoOpenDetectorConfig:
    """Detection thresholds expressed in PWM microseconds and samples."""

    release_pwm: int = 1900
    safe_pwm: int = 1350
    pwm_tolerance_us: int = 20
    required_open_samples: int = 3
    required_closed_samples: int = 3

    def validate(self):
        """Reject unsafe or ambiguous detector configuration."""
        if not 800 <= self.release_pwm <= 2200:
            raise ValueError('release_pwm must be within [800, 2200]')
        if not 800 <= self.safe_pwm <= 2200:
            raise ValueError('safe_pwm must be within [800, 2200]')
        if self.release_pwm == self.safe_pwm:
            raise ValueError('release_pwm and safe_pwm must differ')
        if self.pwm_tolerance_us <= 0:
            raise ValueError('pwm_tolerance_us must be > 0')
        if self.required_open_samples < 1:
            raise ValueError('required_open_samples must be >= 1')
        if self.required_closed_samples < 1:
            raise ValueError('required_closed_samples must be >= 1')


@dataclass(frozen=True)
class ServoOpenEvent:
    """One detector transition for logging by the ROS wrapper."""

    kind: str
    pwm: int
    timestamp: float
    sample_count: int = 0
    duration_s: float = 0.0


class ServoOpenDetector:
    """Detect confirmed openings and require a confirmed close before rearming."""

    def __init__(self, config):
        config.validate()
        self.config = config
        self.state = ServoOpenState.WAITING_FOR_CLOSED
        self.closed_samples = 0
        self.open_samples = 0
        self.candidate_started_at = None
        self.candidate_first_pwm = None

    def _matches(self, pwm, target):
        return abs(int(pwm) - int(target)) <= self.config.pwm_tolerance_us

    def observe(self, pwm, now):
        """Consume one PWM sample and return zero or more transition events."""
        pwm = int(pwm)
        now = float(now)
        events = []

        if self.state == ServoOpenState.WAITING_FOR_CLOSED:
            if self._matches(pwm, self.config.safe_pwm):
                self.closed_samples += 1
                if self.closed_samples >= self.config.required_closed_samples:
                    self.state = ServoOpenState.ARMED
                    events.append(ServoOpenEvent('armed', pwm, now))
            else:
                self.closed_samples = 0
            return events

        if self.state == ServoOpenState.ARMED:
            if not self._matches(pwm, self.config.release_pwm):
                return events
            self.state = ServoOpenState.CANDIDATE
            self.open_samples = 1
            self.candidate_started_at = now
            self.candidate_first_pwm = pwm
            events.append(ServoOpenEvent('candidate_started', pwm, now, 1))
            if self.config.required_open_samples == 1:
                events.extend(self._confirm(pwm, now))
            return events

        if self.state == ServoOpenState.CANDIDATE:
            if self._matches(pwm, self.config.release_pwm):
                self.open_samples += 1
                if self.open_samples >= self.config.required_open_samples:
                    events.extend(self._confirm(pwm, now))
                return events

            events.append(ServoOpenEvent(
                'candidate_rejected',
                pwm,
                now,
                self.open_samples,
                max(0.0, now - self.candidate_started_at),
            ))
            self.state = ServoOpenState.ARMED
            self.open_samples = 0
            self.candidate_started_at = None
            self.candidate_first_pwm = None
            return events

        if self._matches(pwm, self.config.safe_pwm):
            self.closed_samples += 1
            if self.closed_samples >= self.config.required_closed_samples:
                self.state = ServoOpenState.ARMED
                self.closed_samples = 0
                events.append(ServoOpenEvent('rearmed', pwm, now))
        else:
            self.closed_samples = 0
        return events

    def _confirm(self, pwm, now):
        duration_s = max(0.0, now - self.candidate_started_at)
        event = ServoOpenEvent(
            'open_confirmed',
            pwm,
            now,
            self.open_samples,
            duration_s,
        )
        self.state = ServoOpenState.WAITING_FOR_CLOSE
        self.closed_samples = 0
        return [event]
