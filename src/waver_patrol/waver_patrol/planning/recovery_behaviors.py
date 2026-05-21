from __future__ import annotations

from dataclasses import dataclass

from waver_patrol.planning.avoidance_path_generator import AvoidanceAction


@dataclass
class RecoveryTracker:
    max_attempts: int = 3
    attempts: int = 0

    def next_action(self, rear_clear: bool) -> AvoidanceAction:
        self.attempts += 1
        if self.attempts > self.max_attempts:
            return AvoidanceAction.EMERGENCY_STOP
        return AvoidanceAction.BACKUP if rear_clear else AvoidanceAction.WAIT_AND_REPLAN

    def reset(self) -> None:
        self.attempts = 0
