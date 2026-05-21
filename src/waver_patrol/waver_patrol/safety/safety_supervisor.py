from __future__ import annotations

from dataclasses import dataclass, field

from waver_patrol.safety.acceleration_limiter import AccelerationLimiter
from waver_patrol.safety.collision_guard import CollisionDecision
from waver_patrol.safety.command import SafetyEvent, WheelCommand
from waver_patrol.safety.command_mux import CommandMux
from waver_patrol.safety.command_sanitizer import CommandSanitizer
from waver_patrol.safety.localization_guard import LocalizationDecision
from waver_patrol.safety.runaway_guard import RunawayGuard


@dataclass(frozen=True)
class SupervisorDecision:
    command: WheelCommand
    events: list[SafetyEvent] = field(default_factory=list)


class SafetySupervisor:
    def __init__(
        self,
        mux: CommandMux | None = None,
        sanitizer: CommandSanitizer | None = None,
        acceleration: AccelerationLimiter | None = None,
        runaway: RunawayGuard | None = None,
    ):
        self.mux = mux or CommandMux()
        self.sanitizer = sanitizer or CommandSanitizer()
        self.acceleration = acceleration or AccelerationLimiter()
        self.runaway = runaway or RunawayGuard(
            sanitizer=self.sanitizer,
            acceleration_limiter=self.acceleration,
        )

    def update_command(self, command: WheelCommand) -> None:
        self.mux.submit(command)

    def evaluate(
        self,
        *,
        collision: CollisionDecision | None = None,
        localization: LocalizationDecision | None = None,
    ) -> SupervisorDecision:
        events: list[SafetyEvent] = []
        if collision and collision.should_stop:
            events.extend(collision.events)
            return SupervisorDecision(WheelCommand.stop(reason=collision.action.lower()), events)
        if localization and localization.action == "STOP":
            events.extend(localization.events)
            return SupervisorDecision(WheelCommand.stop(reason="localization uncertain"), events)

        selected = self.mux.select()
        events.extend(selected.events)
        runaway = self.runaway.evaluate(selected.command)
        events.extend(runaway.events)
        return SupervisorDecision(runaway.command, events)
