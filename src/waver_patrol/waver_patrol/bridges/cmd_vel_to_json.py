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
    angular_gain: float = 0.65
    max_left_right: float = 0.5
    deadband: float = 0.015


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
        left = self.config.linear_gain * x - self.config.angular_gain * z
        right = self.config.linear_gain * x + self.config.angular_gain * z
        left = clamp(left, -self.config.max_left_right, self.config.max_left_right)
        right = clamp(right, -self.config.max_left_right, self.config.max_left_right)
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
            return sanitized.command
        return self.acceleration_limiter.limit(sanitized.command).command


def twist_msg_to_command(msg: Any, source: str = "nav2") -> WheelCommand:
    twist = TwistCommand(float(msg.linear.x), float(msg.angular.z), source=source)
    return CmdVelToJson().convert(twist, source=source)


def main() -> None:
    print("Use serial_cmd_vel_bridge for ROS2 subscriptions, or import CmdVelToJson in tests.")
