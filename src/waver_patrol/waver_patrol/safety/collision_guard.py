from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from typing import Iterable, Sequence

from waver_patrol.safety.command import SafetyEvent
from waver_patrol.utils import monotonic


SECTORS = {
    "front": (-25.0, 25.0),
    "front_left": (25.0, 65.0),
    "front_right": (-65.0, -25.0),
    "left": (65.0, 120.0),
    "right": (-120.0, -65.0),
    "rear": (140.0, 220.0),
}


@dataclass(frozen=True)
class ScanLike:
    ranges: Sequence[float]
    angle_min: float = -math.pi
    angle_increment: float = math.radians(1.0)
    range_min: float = 0.05
    range_max: float = 12.0
    stamp: float = field(default_factory=monotonic)


@dataclass(frozen=True)
class SectorState:
    min_distance: float = math.inf
    median_distance: float = math.inf
    valid_points: int = 0


@dataclass(frozen=True)
class CollisionDecision:
    action: str
    sectors: dict[str, SectorState]
    speed_limit: float | None = None
    events: list[SafetyEvent] = field(default_factory=list)

    @property
    def should_stop(self) -> bool:
        return self.action in {"HARD_STOP", "EMERGENCY_STOP", "SENSOR_STALE", "SENSOR_DEGRADED"}


@dataclass(frozen=True)
class CollisionConfig:
    hard_stop_distance_m: float = 0.45
    slow_down_distance_m: float = 1.2
    clear_distance_m: float = 1.8
    scan_stale_s: float = 0.5
    min_valid_scan_points: int = 80
    ttc_hard_stop_s: float = 1.5
    ttc_slow_down_s: float = 3.0


class CollisionGuard:
    def __init__(self, config: CollisionConfig | None = None):
        self.config = config or CollisionConfig()

    def evaluate(
        self,
        scan: ScanLike | None,
        *,
        now: float | None = None,
        dynamic_obstacle_front: bool = False,
        ttc_s: float | None = None,
    ) -> CollisionDecision:
        current_time = monotonic() if now is None else now
        events: list[SafetyEvent] = []
        if scan is None or current_time - scan.stamp > self.config.scan_stale_s:
            events.append(SafetyEvent("critical", "scan_stale", "LiDAR scan is stale or missing"))
            return CollisionDecision("SENSOR_STALE", {}, events=events)

        sectors = self.sectorize(scan)
        valid_total = sum(state.valid_points for state in sectors.values())
        if valid_total < self.config.min_valid_scan_points:
            events.append(SafetyEvent("critical", "scan_degraded", "Too few valid LiDAR points"))
            return CollisionDecision("SENSOR_DEGRADED", sectors, events=events)

        front = sectors["front"].min_distance
        if dynamic_obstacle_front and ttc_s is not None:
            if ttc_s < self.config.ttc_hard_stop_s:
                events.append(SafetyEvent("critical", "ttc_hard_stop", "Dynamic obstacle TTC below hard stop"))
                return CollisionDecision("EMERGENCY_STOP", sectors, events=events)
            if ttc_s < self.config.ttc_slow_down_s:
                events.append(SafetyEvent("warning", "ttc_slow_down", "Dynamic obstacle TTC below slow down"))
                return CollisionDecision("SLOW_DOWN", sectors, speed_limit=0.08, events=events)

        if front <= self.config.hard_stop_distance_m:
            events.append(SafetyEvent("critical", "front_hard_stop", "Obstacle inside hard stop zone"))
            return CollisionDecision("HARD_STOP", sectors, events=events)
        if front <= self.config.slow_down_distance_m:
            events.append(SafetyEvent("warning", "front_slow_down", "Obstacle inside slow down zone"))
            return CollisionDecision("SLOW_DOWN", sectors, speed_limit=0.10, events=events)
        return CollisionDecision("CLEAR", sectors, events=events)

    def sectorize(self, scan: ScanLike) -> dict[str, SectorState]:
        buckets: dict[str, list[float]] = {name: [] for name in SECTORS}
        for idx, value in enumerate(scan.ranges):
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                continue
            if value < scan.range_min or value > scan.range_max:
                continue
            angle = math.degrees(scan.angle_min + idx * scan.angle_increment)
            normalized = ((angle + 180.0) % 360.0) - 180.0
            for name, (start, end) in SECTORS.items():
                if name == "rear":
                    if normalized >= 140.0 or normalized <= -140.0:
                        buckets[name].append(float(value))
                elif start <= normalized <= end:
                    buckets[name].append(float(value))
        return {name: self._state(values) for name, values in buckets.items()}

    @staticmethod
    def _state(values: Iterable[float]) -> SectorState:
        values = sorted(values)
        if not values:
            return SectorState()
        trimmed = values[1:-1] if len(values) >= 5 else values
        return SectorState(min(values), statistics.median(trimmed), len(trimmed))
