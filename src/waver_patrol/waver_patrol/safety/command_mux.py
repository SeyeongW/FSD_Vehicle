from __future__ import annotations

from dataclasses import dataclass, field

from waver_patrol.safety.command import SOURCE_PRIORITY, SafetyEvent, WheelCommand
from waver_patrol.utils import monotonic


@dataclass
class CommandSlot:
    command: WheelCommand
    ttl_s: float


@dataclass(frozen=True)
class MuxResult:
    command: WheelCommand
    events: list[SafetyEvent] = field(default_factory=list)


class CommandMux:
    def __init__(self, stale_s: float = 0.3, manual_hold_s: float = 1.0):
        self.stale_s = stale_s
        self.manual_hold_s = manual_hold_s
        self._slots: dict[str, CommandSlot] = {}
        self.manual_override_until = 0.0

    def submit(self, command: WheelCommand, ttl_s: float | None = None) -> None:
        self._slots[command.source] = CommandSlot(command, self.stale_s if ttl_s is None else ttl_s)
        if command.source == "manual":
            self.manual_override_until = max(self.manual_override_until, monotonic() + self.manual_hold_s)

    def clear_non_manual(self) -> None:
        for source in list(self._slots):
            if source != "manual":
                del self._slots[source]

    def clear_all(self) -> None:
        self._slots.clear()

    def select(self, now: float | None = None) -> MuxResult:
        current_time = monotonic() if now is None else now
        events: list[SafetyEvent] = []
        candidates: list[WheelCommand] = []
        for source, slot in list(self._slots.items()):
            age = current_time - slot.command.timestamp
            if age > slot.ttl_s:
                events.append(SafetyEvent("warning", "command_stale", f"{source} command expired"))
                del self._slots[source]
                continue
            candidates.append(slot.command)

        if current_time < self.manual_override_until:
            candidates = [
                cmd for cmd in candidates if cmd.source in {"manual", "safety_stop", "emergency_stop"}
            ]
            if not candidates:
                return MuxResult(WheelCommand.stop(source="safety_stop", reason="manual override hold"), events)

        if not candidates:
            return MuxResult(WheelCommand.stop(source="safety_stop", reason="no fresh command"), events)

        def rank(cmd: WheelCommand) -> int:
            return int(cmd.priority) if int(cmd.priority) > 0 else int(SOURCE_PRIORITY.get(cmd.source, 0))

        return MuxResult(max(candidates, key=rank), events)
