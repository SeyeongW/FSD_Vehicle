from __future__ import annotations


def obstacle_risk_from_distance(distance_m: float, hard_stop_m: float = 0.45, clear_m: float = 1.8) -> float:
    if distance_m <= hard_stop_m:
        return 1.0
    if distance_m >= clear_m:
        return 0.0
    return (clear_m - distance_m) / (clear_m - hard_stop_m)
