from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SpeedZone:
    name: str
    max_speed: float


class KeepoutSpeedZoneManager:
    def __init__(self) -> None:
        self.active_speed_zone: SpeedZone | None = None
        self.keepout_violation = False

    def limit(self, default_speed: float) -> float:
        if self.active_speed_zone is None:
            return default_speed
        return min(default_speed, self.active_speed_zone.max_speed)
