from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from waver_patrol.safety.command import TwistCommand, WheelCommand


Mode = Literal["manual", "patrol", "mapping", "localization_recovery", "sensor_degraded", "speed_zone"]


@dataclass(frozen=True)
class SpeedLimitConfig:
    max_linear_speed: float = 0.28
    max_angular_speed: float = 0.65
    max_patrol_speed: float = 0.20
    max_mapping_speed: float = 0.16
    localization_recovery_max_speed: float = 0.12
    sensor_degraded_max_speed: float = 0.10
    speed_zone_max_speed: float = 0.14


class SpeedLimiter:
    def __init__(self, config: SpeedLimitConfig | None = None):
        self.config = config or SpeedLimitConfig()

    def limit_twist(self, cmd: TwistCommand, mode: Mode = "manual") -> TwistCommand:
        linear_limit = self._linear_limit(mode)
        angular_limit = self.config.max_angular_speed
        return TwistCommand(
            max(-linear_limit, min(linear_limit, cmd.linear_x)),
            max(-angular_limit, min(angular_limit, cmd.angular_z)),
            source=cmd.source,
            timestamp=cmd.timestamp,
        )

    def limit_wheels(self, cmd: WheelCommand, max_abs: float | None = None) -> WheelCommand:
        limit = self.config.max_linear_speed if max_abs is None else max_abs
        peak = max(abs(cmd.left), abs(cmd.right), 1e-9)
        if peak <= limit:
            return cmd
        scale = limit / peak
        return cmd.with_values(cmd.left * scale, cmd.right * scale, reason="speed limited")

    def _linear_limit(self, mode: Mode) -> float:
        if mode == "patrol":
            return self.config.max_patrol_speed
        if mode == "mapping":
            return self.config.max_mapping_speed
        if mode == "localization_recovery":
            return self.config.localization_recovery_max_speed
        if mode == "sensor_degraded":
            return self.config.sensor_degraded_max_speed
        if mode == "speed_zone":
            return self.config.speed_zone_max_speed
        return self.config.max_linear_speed
