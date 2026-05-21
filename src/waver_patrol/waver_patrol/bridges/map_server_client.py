from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MapStatus:
    available: bool
    path: str = ""
    message: str = ""


def require_map(path: str | None) -> MapStatus:
    if not path:
        return MapStatus(False, "", "Patrol requires a map unless explicitly disabled")
    return MapStatus(True, path, "map configured")
