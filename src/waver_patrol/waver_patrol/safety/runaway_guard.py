from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from waver_patrol.safety.acceleration_limiter import AccelerationLimiter
from waver_patrol.safety.command import SafetyEvent, TwistCommand, WheelCommand
from waver_patrol.safety.command_sanitizer import CommandSanitizer
from waver_patrol.utils import finite_float, monotonic


class RunawayAction(str, Enum):
    PASS = "PASS"
    CLAMP = "CLAMP"
    ACCEL_LIMIT = "ACCEL_LIMIT"
    SOFT_STOP = "SOFT_STOP"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass(frozen=True)
class RunawayConfig:
    max_delta_per_tick: float = 0.04
    max_linear_x: float = 0.28
    max_angular_z: float = 0.65
    stale_s: float = 0.3
    imu_accel_spike_mps2: float = 3.0
    yaw_rate_spike_rps: float = 2.0
    keyboard_repeat_hz: float = 30.0


@dataclass(frozen=True)
class RunawayDecision:
    action: RunawayAction
    command: WheelCommand
    events: list[SafetyEvent] = field(default_factory=list)


class RunawayGuard:
    def __init__(
        self,
        config: RunawayConfig | None = None,
        sanitizer: CommandSanitizer | None = None,
        acceleration_limiter: AccelerationLimiter | None = None,
    ):
        self.config = config or RunawayConfig()
        self.sanitizer = sanitizer or CommandSanitizer()
        self.acceleration_limiter = acceleration_limiter or AccelerationLimiter()
        self.previous = WheelCommand.stop(reason="runaway init")

    def evaluate(
        self,
        command: WheelCommand,
        *,
        now: float | None = None,
        twist: TwistCommand | None = None,
        process_duplicate: bool = False,
        serial_reconnecting: bool = False,
        keyboard_repeat_hz: float = 0.0,
        motor_config_changed: bool = False,
        imu_accel_mps2: float | None = None,
        yaw_rate_rps: float | None = None,
    ) -> RunawayDecision:
        current_time = monotonic() if now is None else now
        events: list[SafetyEvent] = []

        try:
            finite_float(command.left, "left")
            finite_float(command.right, "right")
        except ValueError as exc:
            events.append(SafetyEvent("critical", "runaway_invalid_command", str(exc)))
            stop = WheelCommand.stop(source="emergency_stop", reason="invalid command")
            self.previous = stop
            return RunawayDecision(RunawayAction.EMERGENCY_STOP, stop, events)

        valid_sources = {
            "manual",
            "nav2",
            "patrol",
            "patrol_recovery",
            "autonomy",
            "safety_stop",
            "emergency_stop",
        }
        if command.source not in valid_sources:
            events.append(SafetyEvent("critical", "unknown_command_source", "Unknown command source"))
            stop = WheelCommand.stop(source="emergency_stop", reason="unknown source")
            self.previous = stop
            return RunawayDecision(RunawayAction.EMERGENCY_STOP, stop, events)

        if current_time - command.timestamp > self.config.stale_s:
            events.append(SafetyEvent("critical", "command_stale", "Command timestamp is stale"))
            stop = WheelCommand.stop(source="safety_stop", reason="stale command")
            self.previous = stop
            return RunawayDecision(RunawayAction.SOFT_STOP, stop, events)

        if process_duplicate:
            events.append(SafetyEvent("critical", "duplicate_process", "Duplicate command process detected"))
            stop = WheelCommand.stop(source="emergency_stop", reason="duplicate process")
            self.previous = stop
            return RunawayDecision(RunawayAction.EMERGENCY_STOP, stop, events)

        if serial_reconnecting and not command.is_stop:
            events.append(SafetyEvent("critical", "serial_reconnect_replay", "Blocking command during serial reconnect"))
            stop = WheelCommand.stop(source="safety_stop", reason="serial reconnect")
            self.previous = stop
            return RunawayDecision(RunawayAction.SOFT_STOP, stop, events)

        if keyboard_repeat_hz > self.config.keyboard_repeat_hz:
            events.append(SafetyEvent("critical", "keyboard_repeat_flood", "Keyboard repeat rate is too high"))
            stop = WheelCommand.stop(source="emergency_stop", reason="keyboard flood")
            self.previous = stop
            return RunawayDecision(RunawayAction.EMERGENCY_STOP, stop, events)

        if motor_config_changed and not command.is_stop:
            events.append(SafetyEvent("warning", "motor_invert_swap_change", "Motor config changed; require stop"))
            stop = WheelCommand.stop(source="safety_stop", reason="motor config change")
            self.previous = stop
            return RunawayDecision(RunawayAction.SOFT_STOP, stop, events)

        if imu_accel_mps2 is not None and abs(imu_accel_mps2) > self.config.imu_accel_spike_mps2:
            events.append(SafetyEvent("critical", "imu_accel_spike", "IMU acceleration spike"))
            stop = WheelCommand.stop(source="emergency_stop", reason="imu spike")
            self.previous = stop
            return RunawayDecision(RunawayAction.EMERGENCY_STOP, stop, events)

        if yaw_rate_rps is not None and abs(yaw_rate_rps) > self.config.yaw_rate_spike_rps:
            events.append(SafetyEvent("critical", "yaw_rate_spike", "Yaw rate spike"))
            stop = WheelCommand.stop(source="emergency_stop", reason="yaw spike")
            self.previous = stop
            return RunawayDecision(RunawayAction.EMERGENCY_STOP, stop, events)

        if twist is not None:
            if abs(twist.linear_x) > self.config.max_linear_x or abs(twist.angular_z) > self.config.max_angular_z:
                events.append(SafetyEvent("critical", "cmd_vel_spike", "Twist command exceeds configured limits"))
                stop = WheelCommand.stop(source="emergency_stop", reason="cmd_vel spike")
                self.previous = stop
                return RunawayDecision(RunawayAction.EMERGENCY_STOP, stop, events)

        delta = max(abs(command.left - self.previous.left), abs(command.right - self.previous.right))
        if delta > self.config.max_delta_per_tick:
            limited = self.acceleration_limiter.limit(command)
            events.extend(limited.events)
            events.append(SafetyEvent("warning", "wheel_delta_spike", "Wheel command changed too quickly"))
            self.previous = limited.command
            return RunawayDecision(RunawayAction.ACCEL_LIMIT, limited.command, events)

        sanitized = self.sanitizer.sanitize(
            command.left,
            command.right,
            source=command.source,
            reason=command.reason,
            timestamp=current_time,
            priority=int(command.priority),
            is_stop=command.is_stop,
        )
        events.extend(sanitized.events)
        self.previous = sanitized.command
        action = RunawayAction.CLAMP if sanitized.events else RunawayAction.PASS
        return RunawayDecision(action, sanitized.command, events)

    def evaluate_raw(self, left: Any, right: Any, **kwargs: Any) -> RunawayDecision:
        result = self.sanitizer.sanitize(left, right, source=kwargs.pop("source", "unknown"))
        if not result.valid:
            return RunawayDecision(RunawayAction.EMERGENCY_STOP, result.command, result.events)
        return self.evaluate(result.command, **kwargs)
