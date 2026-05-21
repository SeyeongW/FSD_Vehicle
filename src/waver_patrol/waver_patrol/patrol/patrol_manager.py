from __future__ import annotations

from dataclasses import dataclass, field

from waver_patrol.patrol.patrol_route import PatrolRoute
from waver_patrol.patrol.patrol_state_machine import PatrolState, PatrolStateMachine
from waver_patrol.patrol.waypoint_store import PatrolSpec, Waypoint


@dataclass
class PatrolManager:
    spec: PatrolSpec
    state_machine: PatrolStateMachine = field(default_factory=PatrolStateMachine)
    skipped: int = 0
    current_waypoint: Waypoint | None = None

    def route(self) -> PatrolRoute:
        return PatrolRoute(self.spec)

    def start(self, localization_ok: bool) -> PatrolState:
        return self.state_machine.start(localization_ok)

    def next_waypoint_names(self, limit: int = 10) -> list[str]:
        names: list[str] = []
        for idx, waypoint in enumerate(self.route().iter_loop()):
            names.append(waypoint.name)
            if idx + 1 >= limit:
                break
        return names

    def skip_waypoint(self) -> PatrolState:
        self.skipped += 1
        max_skips = int(self.spec.failure_policy.get("return_home_after_skips", 3))
        if self.skipped >= max_skips:
            return self.state_machine.low_battery(critical=False)
        self.state_machine.state = PatrolState.SKIPPING_WAYPOINT
        return self.state_machine.state
