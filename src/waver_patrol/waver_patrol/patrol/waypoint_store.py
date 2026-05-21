from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw_deg: float = 0.0
    dwell_s: float = 0.0
    task: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class PatrolSpec:
    route_name: str
    frame_id: str
    loop_count: int
    mode: str
    start_policy: str
    home: Waypoint
    waypoints: list[Waypoint]
    failure_policy: dict[str, Any] = field(default_factory=dict)


def _waypoint(data: dict[str, Any], default_name: str = "waypoint") -> Waypoint:
    return Waypoint(
        str(data.get("name", default_name)),
        float(data.get("x", 0.0)),
        float(data.get("y", 0.0)),
        float(data.get("yaw_deg", 0.0)),
        float(data.get("dwell_s", 0.0)),
        list(data.get("task", [])),
    )


class WaypointStore:
    @staticmethod
    def load(path: str | Path) -> PatrolSpec:
        try:
            import yaml
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("PyYAML is required for waypoint files") from exc
        waypoint_path = Path(path).expanduser()
        if not waypoint_path.exists() and not waypoint_path.is_absolute():
            try:
                from ament_index_python.packages import get_package_share_directory

                share_path = Path(get_package_share_directory("waver_patrol")) / waypoint_path
                if share_path.exists():
                    waypoint_path = share_path
            except Exception:
                pass
        with waypoint_path.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}
        waypoints = [_waypoint(item, f"wp_{idx}") for idx, item in enumerate(data.get("waypoints", []))]
        home = _waypoint(data.get("home", {"name": "home"}), "home")
        return PatrolSpec(
            str(data.get("route_name", "unnamed")),
            str(data.get("frame_id", "map")),
            int(data.get("loop_count", -1)),
            str(data.get("mode", "loop")),
            str(data.get("start_policy", "nearest_first")),
            home,
            waypoints,
            dict(data.get("failure_policy", {})),
        )

    @staticmethod
    def save(path: str | Path, spec: PatrolSpec) -> None:
        try:
            import yaml
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("PyYAML is required for waypoint files") from exc
        data = {
            "route_name": spec.route_name,
            "frame_id": spec.frame_id,
            "loop_count": spec.loop_count,
            "mode": spec.mode,
            "start_policy": spec.start_policy,
            "home": spec.home.__dict__,
            "waypoints": [wp.__dict__ for wp in spec.waypoints],
            "failure_policy": spec.failure_policy,
        }
        with Path(path).expanduser().open("w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
