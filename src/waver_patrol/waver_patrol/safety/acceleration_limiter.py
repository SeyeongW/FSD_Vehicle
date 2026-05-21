from __future__ import annotations

from dataclasses import dataclass, field

from waver_patrol.safety.command import SafetyEvent, WheelCommand
from waver_patrol.utils import clamp, monotonic, sign


@dataclass(frozen=True)
class AccelerationLimiterConfig:
    max_delta_per_tick: float = 0.04
    jerk_delta_per_tick: float = 0.08
    require_neutral_before_reverse: bool = True


@dataclass(frozen=True)
class AccelerationResult:
    command: WheelCommand
    events: list[SafetyEvent] = field(default_factory=list)
    inserted_neutral: bool = False


class AccelerationLimiter:
    def __init__(self, config: AccelerationLimiterConfig | None = None):
        self.config = config or AccelerationLimiterConfig()
        self.previous = WheelCommand.stop(reason="initial")
        self.previous_delta_left = 0.0
        self.previous_delta_right = 0.0

    def reset(self) -> None:
        self.previous = WheelCommand.stop(reason="accel limiter reset")
        self.previous_delta_left = 0.0
        self.previous_delta_right = 0.0

    def limit(self, command: WheelCommand) -> AccelerationResult:
        events: list[SafetyEvent] = []
        if command.is_stop:
            self.previous = WheelCommand.stop(source=command.source, reason=command.reason or "stop")
            return AccelerationResult(self.previous, events)

        if self.config.require_neutral_before_reverse:
            left_reverse = sign(self.previous.left) != 0 and sign(command.left) == -sign(self.previous.left)
            right_reverse = sign(self.previous.right) != 0 and sign(command.right) == -sign(self.previous.right)
            if left_reverse or right_reverse:
                neutral = WheelCommand.stop(source=command.source, reason="neutral before reverse")
                self.previous = neutral
                events.append(SafetyEvent("warning", "neutral_before_reverse", "Inserted neutral stop"))
                return AccelerationResult(neutral, events, inserted_neutral=True)

        new_left = self._limit_axis(command.left, self.previous.left, "left", events)
        new_right = self._limit_axis(command.right, self.previous.right, "right", events)
        limited = WheelCommand(
            new_left,
            new_right,
            source=command.source,
            timestamp=monotonic(),
            priority=command.priority,
            reason=command.reason or "acceleration limited",
            is_stop=abs(new_left) < 1e-9 and abs(new_right) < 1e-9,
        )
        self.previous_delta_left = limited.left - self.previous.left
        self.previous_delta_right = limited.right - self.previous.right
        self.previous = limited
        return AccelerationResult(limited, events)

    def _limit_axis(
        self,
        desired: float,
        previous: float,
        axis: str,
        events: list[SafetyEvent],
    ) -> float:
        delta = desired - previous
        limited_delta = clamp(delta, -self.config.max_delta_per_tick, self.config.max_delta_per_tick)
        if abs(limited_delta - delta) > 1e-9:
            events.append(
                SafetyEvent(
                    "warning",
                    "acceleration_limited",
                    f"{axis} command delta limited",
                    data={"desired_delta": delta, "limited_delta": limited_delta},
                )
            )
        return previous + limited_delta
