from __future__ import annotations

from dataclasses import dataclass


@dataclass
class EStop:
    require_manual_reset: bool = True
    active: bool = False
    reason: str = ""

    def trigger(self, reason: str = "operator") -> None:
        self.active = True
        self.reason = reason

    def reset(self) -> bool:
        self.active = False
        self.reason = ""
        return True
