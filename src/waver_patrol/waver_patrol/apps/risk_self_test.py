from __future__ import annotations

from waver_patrol.safety.command import TwistCommand, WheelCommand
from waver_patrol.safety.runaway_guard import RunawayGuard


def main() -> None:
    guard = RunawayGuard()
    print(guard.evaluate(WheelCommand(0.5, 0.5, source="manual")).action.value)
    print(guard.evaluate_raw(float("nan"), 0.0, source="manual").action.value)
    print(guard.evaluate(WheelCommand(0.0, 0.0, source="nav2"), twist=TwistCommand(2.0, 0.0)).action.value)
