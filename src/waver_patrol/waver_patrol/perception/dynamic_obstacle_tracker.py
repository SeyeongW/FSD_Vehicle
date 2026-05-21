from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DynamicObstacleTracker:
    dynamic_obstacle_front: bool = False

    def update_from_ttc(self, ttc_s: float | None) -> bool:
        self.dynamic_obstacle_front = ttc_s is not None and ttc_s < 3.0
        return self.dynamic_obstacle_front
