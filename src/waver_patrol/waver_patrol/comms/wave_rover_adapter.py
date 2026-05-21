from __future__ import annotations

from dataclasses import dataclass

from waver_patrol.comms.serial_json_client import SerialJsonClient
from waver_patrol.safety.acceleration_limiter import AccelerationLimiter
from waver_patrol.safety.command import WheelCommand
from waver_patrol.safety.command_sanitizer import CommandSanitizer
from waver_patrol.safety.runaway_guard import RunawayGuard


@dataclass
class WaveRoverAdapter:
    client: SerialJsonClient
    sanitizer: CommandSanitizer
    acceleration_limiter: AccelerationLimiter
    runaway_guard: RunawayGuard

    def send(self, command: WheelCommand) -> WheelCommand:
        sanitized = self.sanitizer.sanitize(
            command.left,
            command.right,
            source=command.source,
            reason=command.reason,
            timestamp=command.timestamp,
            priority=int(command.priority),
            is_stop=command.is_stop,
        )
        if not sanitized.valid:
            safe = WheelCommand.stop(source="emergency_stop", reason="sanitize failed")
            self.client.send_command(safe)
            return safe
        runaway = self.runaway_guard.evaluate(sanitized.command)
        safe_cmd = runaway.command
        if not safe_cmd.is_stop:
            safe_cmd = self.acceleration_limiter.limit(safe_cmd).command
        self.client.send_command(safe_cmd)
        return safe_cmd

    def stop(self, repeat: int = 5) -> None:
        self.client.send_stop(repeat=repeat)
