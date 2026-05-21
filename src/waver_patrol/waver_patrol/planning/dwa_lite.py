from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class VelocitySample:
    linear: float
    angular: float
    cost: float


class DwaLite:
    def __init__(self, short_horizon_s: float = 0.8):
        self.short_horizon_s = short_horizon_s

    def choose(
        self,
        obstacle_risk: float,
        heading_error: float,
        previous_linear: float = 0.0,
        pose_uncertainty: float = 0.0,
    ) -> VelocitySample:
        candidates = [
            VelocitySample(0.08, 0.0, 0.0),
            VelocitySample(0.06, 0.3, 0.0),
            VelocitySample(0.06, -0.3, 0.0),
            VelocitySample(0.0, 0.4, 0.0),
            VelocitySample(0.0, -0.4, 0.0),
        ]
        scored: list[VelocitySample] = []
        for sample in candidates:
            reverse_penalty = 5.0 if sample.linear < 0.0 else 0.0
            smoothness_cost = abs(sample.linear - previous_linear)
            cost = (
                4.0 * obstacle_risk
                + abs(heading_error - sample.angular)
                + smoothness_cost
                + reverse_penalty
                + 2.0 * pose_uncertainty
            )
            scored.append(VelocitySample(sample.linear, sample.angular, cost))
        return min(scored, key=lambda sample: sample.cost)
