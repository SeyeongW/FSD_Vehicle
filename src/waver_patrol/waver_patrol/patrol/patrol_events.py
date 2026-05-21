from __future__ import annotations

from dataclasses import dataclass, field

from waver_patrol.utils import monotonic


@dataclass(frozen=True)
class PatrolEvent:
    event: str
    waypoint: str = ""
    timestamp: float = field(default_factory=monotonic)
    detail: str = ""
