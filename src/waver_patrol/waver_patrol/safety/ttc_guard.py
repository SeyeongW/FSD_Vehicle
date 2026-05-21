from __future__ import annotations

import math
from dataclasses import dataclass, field

from waver_patrol.safety.command import SafetyEvent
from waver_patrol.utils import monotonic


@dataclass(frozen=True)
class TtcConfig:
    hard_stop_s: float = 1.5
    slow_down_s: float = 3.0
    smoothing_alpha: float = 0.45


@dataclass(frozen=True)
class TtcDecision:
    action: str
    ttc_s: float | None
    approaching_speed_mps: float
    dynamic_obstacle: bool
    events: list[SafetyEvent] = field(default_factory=list)


class TtcGuard:
    def __init__(self, config: TtcConfig | None = None):
        self.config = config or TtcConfig()
        self.previous_distance: float | None = None
        self.previous_time: float | None = None
        self.smoothed_speed = 0.0

    def update(self, distance_m: float | None, *, now: float | None = None) -> TtcDecision:
        current_time = monotonic() if now is None else now
        events: list[SafetyEvent] = []
        if distance_m is None or not math.isfinite(distance_m) or distance_m <= 0.0:
            events.append(SafetyEvent("warning", "ttc_invalid_distance", "No valid distance for TTC"))
            return TtcDecision("UNKNOWN", None, 0.0, False, events)

        if self.previous_distance is None or self.previous_time is None:
            self.previous_distance = distance_m
            self.previous_time = current_time
            return TtcDecision("CLEAR", None, 0.0, False, events)

        dt = max(current_time - self.previous_time, 1e-6)
        raw_speed = (self.previous_distance - distance_m) / dt
        self.smoothed_speed = (
            self.config.smoothing_alpha * raw_speed
            + (1.0 - self.config.smoothing_alpha) * self.smoothed_speed
        )
        self.previous_distance = distance_m
        self.previous_time = current_time

        if self.smoothed_speed <= 0.02:
            return TtcDecision("CLEAR", None, self.smoothed_speed, False, events)

        ttc = distance_m / self.smoothed_speed
        dynamic = self.smoothed_speed > 0.05
        if ttc < self.config.hard_stop_s:
            events.append(SafetyEvent("critical", "ttc_hard_stop", "Approaching obstacle TTC is critical"))
            return TtcDecision("EMERGENCY_STOP", ttc, self.smoothed_speed, dynamic, events)
        if ttc < self.config.slow_down_s:
            events.append(SafetyEvent("warning", "ttc_slow_down", "Approaching obstacle TTC is low"))
            return TtcDecision("SLOW_DOWN", ttc, self.smoothed_speed, dynamic, events)
        return TtcDecision("CLEAR", ttc, self.smoothed_speed, dynamic, events)
