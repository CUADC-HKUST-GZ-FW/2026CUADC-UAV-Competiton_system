"""Keep a short wall-clock history of aircraft lat/lon/altitude."""

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional


@dataclass
class TelemetrySample:
    wall_time: float
    relative_alt_m: Optional[float] = None
    latitude: Optional[float] = None
    longitude: Optional[float] = None
    amsl_m: Optional[float] = None
    source: str = 'none'


class TelemetryBuffer:
    """Merge independently arriving MAVROS position topics into one stamped sample."""

    def __init__(self, maxlen=200):
        self._history: Deque[TelemetrySample] = deque(maxlen=maxlen)
        self._latest = TelemetrySample(wall_time=0.0)
        self.rel_alt_count = 0
        self.global_count = 0

    @property
    def latest(self):
        return self._latest

    def _commit(self, now, source):
        sample = TelemetrySample(
            wall_time=now,
            relative_alt_m=self._latest.relative_alt_m,
            latitude=self._latest.latitude,
            longitude=self._latest.longitude,
            amsl_m=self._latest.amsl_m,
            source=source,
        )
        self._latest = sample
        self._history.append(sample)
        return sample

    def update_rel_alt(self, now, relative_alt_m):
        self._latest.relative_alt_m = float(relative_alt_m)
        self.rel_alt_count += 1
        return self._commit(now, 'rel_alt')

    def update_global(self, now, latitude, longitude, amsl_m):
        self._latest.latitude = float(latitude)
        self._latest.longitude = float(longitude)
        self._latest.amsl_m = float(amsl_m)
        self.global_count += 1
        return self._commit(now, 'global')

    def nearest(self, wall_time, max_age_sec=0.25):
        if not self._history:
            return None, None
        sample = min(self._history, key=lambda item: abs(item.wall_time - wall_time))
        age = wall_time - sample.wall_time
        if abs(age) > max_age_sec:
            return sample, age
        return sample, age

    def age(self, now):
        if self._latest.wall_time <= 0.0:
            return None
        return now - self._latest.wall_time
