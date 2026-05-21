from __future__ import annotations

from dataclasses import dataclass, field

from waver_patrol.safety.command import SafetyEvent
from waver_patrol.utils import monotonic


@dataclass(frozen=True)
class SensorStaleDecision:
    ok: bool
    stale: list[str] = field(default_factory=list)
    events: list[SafetyEvent] = field(default_factory=list)


class SensorStaleGuard:
    def __init__(self, thresholds_s: dict[str, float] | None = None):
        self.thresholds_s = thresholds_s or {"scan": 0.5, "camera": 1.0, "pose": 1.0, "tf": 1.0}
        self.last_seen: dict[str, float] = {}

    def mark(self, sensor: str, stamp: float | None = None) -> None:
        self.last_seen[sensor] = monotonic() if stamp is None else stamp

    def evaluate(self, now: float | None = None) -> SensorStaleDecision:
        current_time = monotonic() if now is None else now
        stale: list[str] = []
        events: list[SafetyEvent] = []
        for sensor, threshold in self.thresholds_s.items():
            if sensor not in self.last_seen or current_time - self.last_seen[sensor] > threshold:
                stale.append(sensor)
                events.append(SafetyEvent("critical", f"{sensor}_stale", f"{sensor} is stale"))
        return SensorStaleDecision(not stale, stale, events)
