from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from waver_patrol.safety.acceleration_limiter import AccelerationLimiter
from waver_patrol.safety.command import TwistCommand, WheelCommand
from waver_patrol.safety.command_sanitizer import CommandSanitizer
from waver_patrol.utils import clamp


@dataclass(frozen=True)
class CmdVelToJsonConfig:
    linear_gain: float = 1.0
    angular_gain: float = 0.55
    max_left_right: float = 0.32
    deadband: float = 0.015
    min_linear_ratio: float = 0.25
    pure_turn_mode: str = "pivot"
    pure_turn_linear_epsilon: float = 0.01
    pure_turn_min_ratio: float = 0.16
    pure_turn_max_ratio: float = 0.16
    mixed_turn_mode: str = "inside_brake"
    mixed_turn_inner_ratio: float = 0.0
    mixed_turn_outer_ratio: float = 0.22


class CmdVelToJson:
    def __init__(
        self,
        config: CmdVelToJsonConfig | None = None,
        sanitizer: CommandSanitizer | None = None,
        acceleration_limiter: AccelerationLimiter | None = None,
    ):
        self.config = config or CmdVelToJsonConfig()
        self.sanitizer = sanitizer or CommandSanitizer()
        self.acceleration_limiter = acceleration_limiter or AccelerationLimiter()

    def convert(self, twist: TwistCommand, source: str | None = None) -> WheelCommand:
        x = 0.0 if abs(twist.linear_x) < self.config.deadband else twist.linear_x
        z = 0.0 if abs(twist.angular_z) < self.config.deadband else twist.angular_z
        if abs(x) <= self.config.pure_turn_linear_epsilon and abs(z) >= self.config.deadband:
            left, right = self._pure_turn(z)
        elif (
            self.config.mixed_turn_mode.strip().lower()
            in {"same_direction_arc", "inside_brake", "brake_arc", "skid_arc"}
            and abs(x) > self.config.pure_turn_linear_epsilon
            and abs(z) >= self.config.deadband
        ):
            left, right = self._mixed_turn(x, z)
        else:
            left = self.config.linear_gain * x - self.config.angular_gain * z
            right = self.config.linear_gain * x + self.config.angular_gain * z
            left = clamp(left, -self.config.max_left_right, self.config.max_left_right)
            right = clamp(right, -self.config.max_left_right, self.config.max_left_right)
            if abs(x) > self.config.deadband and abs(z) < self.config.deadband:
                left = self._with_min_ratio(left)
                right = self._with_min_ratio(right)
        is_stop = abs(left) < 1e-9 and abs(right) < 1e-9
        sanitized = self.sanitizer.sanitize(
            left,
            right,
            source=source or twist.source,
            reason="cmd_vel",
            timestamp=twist.timestamp,
            is_stop=is_stop,
        )
        if not sanitized.valid:
            return sanitized.command
        if is_stop:
            self.acceleration_limiter.reset()
            return sanitized.command
        return self.acceleration_limiter.limit(sanitized.command).command

    def _pure_turn(self, angular: float) -> tuple[float, float]:
        turn_limit = min(max(self.config.pure_turn_max_ratio, 0.0), self.config.max_left_right)
        turn = abs(self.config.angular_gain * angular)
        turn = clamp(turn, min(self.config.pure_turn_min_ratio, turn_limit), turn_limit)
        mode = self.config.pure_turn_mode.strip().lower()
        if mode == "pivot":
            return (0.0, turn) if angular > 0.0 else (turn, 0.0)
        if angular > 0.0:
            return -turn, turn
        return turn, -turn

    def _mixed_turn(self, linear: float, angular: float) -> tuple[float, float]:
        mode = self.config.mixed_turn_mode.strip().lower()
        limit = min(
            self.config.max_left_right,
            max(self.config.mixed_turn_outer_ratio, abs(self.config.linear_gain * linear)),
        )
        outer_mag = clamp(
            max(abs(self.config.linear_gain * linear), self.config.mixed_turn_outer_ratio),
            self.config.deadband,
            limit,
        )
        sign = 1.0 if linear > 0.0 else -1.0
        outer = sign * outer_mag
        if mode in {"inside_brake", "brake_arc", "skid_arc"}:
            inner = sign * clamp(
                self.config.mixed_turn_inner_ratio,
                -outer_mag,
                outer_mag,
            )
        else:
            inner_mag = clamp(self.config.mixed_turn_inner_ratio, self.config.deadband, outer_mag)
            inner = sign * inner_mag
        if angular > 0.0:
            return (inner, outer) if linear > 0.0 else (outer, inner)
        return (outer, inner) if linear > 0.0 else (inner, outer)

    def _with_min_ratio(self, value: float) -> float:
        if abs(value) < self.config.deadband:
            return 0.0
        minimum = min(max(self.config.min_linear_ratio, 0.0), self.config.max_left_right)
        if value > 0.0:
            return clamp(max(value, minimum), -self.config.max_left_right, self.config.max_left_right)
        return clamp(min(value, -minimum), -self.config.max_left_right, self.config.max_left_right)


def twist_msg_to_command(msg: Any, source: str = "nav2") -> WheelCommand:
    twist = TwistCommand(float(msg.linear.x), float(msg.angular.z), source=source)
    return CmdVelToJson().convert(twist, source=source)


def main() -> None:
    print("Use serial_cmd_vel_bridge for ROS2 subscriptions, or import CmdVelToJson in tests.")
