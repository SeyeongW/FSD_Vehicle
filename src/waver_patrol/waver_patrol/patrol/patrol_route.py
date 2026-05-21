from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterator

from waver_patrol.patrol.waypoint_store import PatrolSpec, Waypoint


@dataclass
class PatrolRoute:
    spec: PatrolSpec
    current_index: int = 0
    direction: int = 1

    def ordered_waypoints(self, current_pose_xy: tuple[float, float] | None = None) -> list[Waypoint]:
        waypoints = list(self.spec.waypoints)
        if self.spec.start_policy == "nearest_first" and current_pose_xy and waypoints:
            cx, cy = current_pose_xy
            nearest = min(range(len(waypoints)), key=lambda idx: math.hypot(waypoints[idx].x - cx, waypoints[idx].y - cy))
            waypoints = waypoints[nearest:] + waypoints[:nearest]
        if self.spec.mode == "reverse":
            waypoints.reverse()
        return waypoints

    def iter_once(self) -> Iterator[Waypoint]:
        waypoints = self.ordered_waypoints()
        if self.spec.mode == "pingpong" and len(waypoints) > 1:
            yield from waypoints
            yield from waypoints[-2:0:-1]
            return
        yield from waypoints

    def iter_loop(self) -> Iterator[Waypoint]:
        loops = self.spec.loop_count
        count = 0
        while loops < 0 or count < loops:
            yield from self.iter_once()
            count += 1
