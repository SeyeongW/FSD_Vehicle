from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from waver_patrol.safety.collision_guard import SectorState


class AvoidanceAction(str, Enum):
    STOP = "STOP"
    SLOW_FORWARD = "SLOW_FORWARD"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    ARC_LEFT = "ARC_LEFT"
    ARC_RIGHT = "ARC_RIGHT"
    BACKUP = "BACKUP"
    WAIT_AND_REPLAN = "WAIT_AND_REPLAN"
    CANCEL_GOAL = "CANCEL_GOAL"
    RETURN_HOME = "RETURN_HOME"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass(frozen=True)
class AvoidanceInput:
    sectors: dict[str, SectorState]
    blocked_duration_s: float = 0.0
    recovery_attempts: int = 0
    max_recovery_attempts: int = 3
    rear_clear: bool = True
    keepout_violation: bool = False
    localization_covariance_xy: float = 0.0
    dynamic_obstacle: bool = False


@dataclass(frozen=True)
class AvoidanceDecision:
    action: AvoidanceAction
    reason: str
    max_duration_s: float | None = None
    metadata: dict[str, float | str] = field(default_factory=dict)


class AvoidancePathGenerator:
    def __init__(self, hard_stop_m: float = 0.45, clear_m: float = 1.8):
        self.hard_stop_m = hard_stop_m
        self.clear_m = clear_m

    def decide(self, data: AvoidanceInput) -> AvoidanceDecision:
        if data.recovery_attempts > data.max_recovery_attempts:
            return AvoidanceDecision(AvoidanceAction.EMERGENCY_STOP, "recovery attempts exceeded")
        if data.keepout_violation:
            return AvoidanceDecision(AvoidanceAction.STOP, "candidate path enters keepout zone")
        if data.localization_covariance_xy > 1.5:
            return AvoidanceDecision(AvoidanceAction.CANCEL_GOAL, "pose uncertainty too high")

        front = data.sectors.get("front", SectorState()).min_distance
        left = min(
            data.sectors.get("front_left", SectorState()).min_distance,
            data.sectors.get("left", SectorState()).min_distance,
        )
        right = min(
            data.sectors.get("front_right", SectorState()).min_distance,
            data.sectors.get("right", SectorState()).min_distance,
        )
        rear = data.sectors.get("rear", SectorState()).min_distance
        rear_clear = data.rear_clear and rear >= self.clear_m

        if data.dynamic_obstacle:
            return AvoidanceDecision(AvoidanceAction.STOP, "dynamic obstacle; wait before replanning")
        if not math.isfinite(front) or front < self.hard_stop_m:
            if left > self.clear_m and left >= right:
                return AvoidanceDecision(AvoidanceAction.ARC_LEFT, "front blocked; left sector clearer")
            if right > self.clear_m:
                return AvoidanceDecision(AvoidanceAction.ARC_RIGHT, "front blocked; right sector clearer")
            if data.blocked_duration_s > 5.0 and rear_clear:
                return AvoidanceDecision(AvoidanceAction.BACKUP, "blocked too long; rear clear", 0.8)
            if data.blocked_duration_s > 2.0:
                return AvoidanceDecision(AvoidanceAction.CANCEL_GOAL, "blocked; cancel goal and replan")
            return AvoidanceDecision(AvoidanceAction.WAIT_AND_REPLAN, "all local paths blocked")
        if front < self.clear_m:
            return AvoidanceDecision(AvoidanceAction.SLOW_FORWARD, "front not fully clear")
        return AvoidanceDecision(AvoidanceAction.SLOW_FORWARD, "path clear")
