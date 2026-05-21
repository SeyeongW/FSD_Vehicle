from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class BatteryGuard:
    low_battery_v: float = 9.6
    critical_battery_v: float = 9.0

    def evaluate(self, voltage: float | None) -> str:
        if voltage is None or voltage <= 0.1:
            return "UNKNOWN"
        if voltage <= self.critical_battery_v:
            return "CRITICAL"
        if voltage <= self.low_battery_v:
            return "LOW"
        return "OK"
