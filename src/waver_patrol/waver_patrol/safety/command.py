from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any

from waver_patrol.utils import monotonic


class CommandPriority(IntEnum):
    AUTONOMY = 10
    NAV2 = 20
    PATROL_RECOVERY = 30
    MANUAL = 40
    EMERGENCY_STOP = 50
    SAFETY_STOP = 60


SOURCE_PRIORITY = {
    "autonomy": CommandPriority.AUTONOMY,
    "patrol": CommandPriority.AUTONOMY,
    "nav2": CommandPriority.NAV2,
    "patrol_recovery": CommandPriority.PATROL_RECOVERY,
    "manual": CommandPriority.MANUAL,
    "emergency_stop": CommandPriority.EMERGENCY_STOP,
    "safety_stop": CommandPriority.SAFETY_STOP,
}


@dataclass(frozen=True)
class WheelCommand:
    left: float
    right: float
    source: str = "unknown"
    timestamp: float = field(default_factory=monotonic)
    priority: CommandPriority | int = CommandPriority.AUTONOMY
    reason: str = ""
    is_stop: bool = False

    @classmethod
    def stop(cls, source: str = "safety_stop", reason: str = "stop") -> "WheelCommand":
        priority = SOURCE_PRIORITY.get(source, CommandPriority.SAFETY_STOP)
        return cls(0.0, 0.0, source=source, priority=priority, reason=reason, is_stop=True)

    def with_values(self, left: float, right: float, reason: str | None = None) -> "WheelCommand":
        return WheelCommand(
            left=left,
            right=right,
            source=self.source,
            timestamp=monotonic(),
            priority=self.priority,
            reason=self.reason if reason is None else reason,
            is_stop=abs(left) < 1e-9 and abs(right) < 1e-9,
        )

    def as_rover_json(self) -> dict[str, Any]:
        return {"T": 1, "L": self.left, "R": self.right}


@dataclass(frozen=True)
class SafetyEvent:
    level: str
    code: str
    message: str
    timestamp: float = field(default_factory=monotonic)
    data: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class TwistCommand:
    linear_x: float
    angular_z: float
    source: str = "nav2"
    timestamp: float = field(default_factory=monotonic)
