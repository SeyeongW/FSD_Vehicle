from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from waver_patrol.safety.command import SafetyEvent, SOURCE_PRIORITY, WheelCommand
from waver_patrol.utils import clamp, finite_float, monotonic


@dataclass(frozen=True)
class SanitizerConfig:
    max_left_right: float = 0.5
    max_demo_speed: float = 0.28
    invert_left: bool = False
    invert_right: bool = False
    swap_left_right: bool = False


@dataclass(frozen=True)
class SanitizerResult:
    command: WheelCommand
    valid: bool
    events: list[SafetyEvent] = field(default_factory=list)


class CommandSanitizer:
    def __init__(self, config: SanitizerConfig | None = None):
        self.config = config or SanitizerConfig()

    @classmethod
    def from_mapping(cls, rover: dict[str, Any], motor: dict[str, Any]) -> "CommandSanitizer":
        return cls(
            SanitizerConfig(
                max_left_right=float(rover.get("max_left_right", 0.5)),
                max_demo_speed=float(rover.get("max_demo_speed", 0.28)),
                invert_left=bool(motor.get("invert_left", False)),
                invert_right=bool(motor.get("invert_right", False)),
                swap_left_right=bool(motor.get("swap_left_right", False)),
            )
        )

    def sanitize(
        self,
        left: Any,
        right: Any,
        *,
        source: str = "unknown",
        reason: str = "",
        timestamp: float | None = None,
        priority: int | None = None,
        is_stop: bool = False,
    ) -> SanitizerResult:
        events: list[SafetyEvent] = []
        if is_stop:
            return SanitizerResult(
                WheelCommand.stop(
                    source=source if source != "unknown" else "safety_stop",
                    reason=reason or "stop",
                ),
                True,
                events,
            )
        try:
            raw_left = finite_float(left, "left")
            raw_right = finite_float(right, "right")
        except ValueError as exc:
            events.append(SafetyEvent("critical", "invalid_command", str(exc), data={"source": source}))
            return SanitizerResult(
                WheelCommand.stop(source="emergency_stop", reason="invalid command"),
                False,
                events,
            )

        max_output = min(self.config.max_left_right, self.config.max_demo_speed)
        clamped_left = clamp(raw_left, -max_output, max_output)
        clamped_right = clamp(raw_right, -max_output, max_output)
        if (clamped_left, clamped_right) != (raw_left, raw_right):
            events.append(
                SafetyEvent(
                    "warning",
                    "command_clamped",
                    "Wheel command exceeded configured limits",
                    data={"left": raw_left, "right": raw_right, "max": max_output},
                )
            )

        if self.config.invert_left:
            clamped_left = -clamped_left
        if self.config.invert_right:
            clamped_right = -clamped_right
        if self.config.swap_left_right:
            clamped_left, clamped_right = clamped_right, clamped_left

        return SanitizerResult(
            WheelCommand(
                clamped_left,
                clamped_right,
                source=source,
                timestamp=timestamp if timestamp is not None else monotonic(),
                priority=priority if priority is not None else SOURCE_PRIORITY.get(source, 0),
                reason=reason,
                is_stop=abs(clamped_left) < 1e-9 and abs(clamped_right) < 1e-9,
            ),
            True,
            events,
        )
