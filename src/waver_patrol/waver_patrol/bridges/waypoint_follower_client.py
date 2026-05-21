from __future__ import annotations

from dataclasses import dataclass

from waver_patrol.bridges.nav2_client import Nav2Client
from waver_patrol.patrol.patrol_route import PatrolRoute


@dataclass
class WaypointFollowerClient:
    nav2: Nav2Client

    def follow_once(self, route: PatrolRoute) -> list[str]:
        visited: list[str] = []
        for waypoint in route.iter_once():
            self.nav2.send_goal(waypoint)
            visited.append(waypoint.name)
        return visited
