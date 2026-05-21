from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class PatrolState(str, Enum):
    IDLE = "IDLE"
    STARTUP_CHECK = "STARTUP_CHECK"
    WAIT_FOR_LOCALIZATION = "WAIT_FOR_LOCALIZATION"
    PATROL_READY = "PATROL_READY"
    GOING_TO_WAYPOINT = "GOING_TO_WAYPOINT"
    ARRIVED_AT_WAYPOINT = "ARRIVED_AT_WAYPOINT"
    WAYPOINT_TASK = "WAYPOINT_TASK"
    AVOIDING_OBSTACLE = "AVOIDING_OBSTACLE"
    WAITING_FOR_CLEAR_PATH = "WAITING_FOR_CLEAR_PATH"
    REPLANNING = "REPLANNING"
    RECOVERY = "RECOVERY"
    SKIPPING_WAYPOINT = "SKIPPING_WAYPOINT"
    RETURNING_HOME = "RETURNING_HOME"
    MANUAL_OVERRIDE = "MANUAL_OVERRIDE"
    SENSOR_DEGRADED = "SENSOR_DEGRADED"
    POSE_UNCERTAIN = "POSE_UNCERTAIN"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    LOW_BATTERY = "LOW_BATTERY"
    SHUTDOWN = "SHUTDOWN"


@dataclass
class PatrolStateMachine:
    state: PatrolState = PatrolState.IDLE
    recovery_attempts: int = 0
    max_recovery_attempts: int = 3

    def start(self, localization_ok: bool) -> PatrolState:
        self.state = PatrolState.PATROL_READY if localization_ok else PatrolState.WAIT_FOR_LOCALIZATION
        return self.state

    def manual_override(self) -> PatrolState:
        self.state = PatrolState.MANUAL_OVERRIDE
        return self.state

    def reset_manual(self, localization_ok: bool = True) -> PatrolState:
        self.state = PatrolState.PATROL_READY if localization_ok else PatrolState.WAIT_FOR_LOCALIZATION
        return self.state

    def obstacle(self, hard: bool, dynamic: bool = False) -> PatrolState:
        if dynamic:
            self.state = PatrolState.WAITING_FOR_CLEAR_PATH
        elif hard:
            self.state = PatrolState.AVOIDING_OBSTACLE
        else:
            self.state = PatrolState.GOING_TO_WAYPOINT
        return self.state

    def recovery_failed(self) -> PatrolState:
        self.recovery_attempts += 1
        if self.recovery_attempts > self.max_recovery_attempts:
            self.state = PatrolState.EMERGENCY_STOP
        else:
            self.state = PatrolState.RECOVERY
        return self.state

    def low_battery(self, critical: bool) -> PatrolState:
        self.state = PatrolState.EMERGENCY_STOP if critical else PatrolState.LOW_BATTERY
        return self.state
