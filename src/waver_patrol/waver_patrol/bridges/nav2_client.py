from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class Nav2GoalStatus:
    active: bool = False
    cancelled: bool = False
    message: str = ""


class Nav2Client:
    def __init__(self) -> None:
        self.status = Nav2GoalStatus()
        self._client: Any | None = None

    def cancel_on_safety(self, reason: str) -> None:
        self.status.cancelled = True
        self.status.active = False
        self.status.message = reason

    def send_goal(self, *_args: Any, **_kwargs: Any) -> Nav2GoalStatus:
        self.status = Nav2GoalStatus(active=True, cancelled=False, message="goal sent")
        return self.status

    def cancel_goal(self, reason: str = "cancelled") -> Nav2GoalStatus:
        self.status = Nav2GoalStatus(active=False, cancelled=True, message=reason)
        return self.status
