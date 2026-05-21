from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PathCandidate:
    name: str
    min_distance_m: float
    ttc_s: float | None = None
    keepout_violation: bool = False


class PathValidator:
    def __init__(self, min_distance_m: float = 0.45, min_ttc_s: float = 1.5):
        self.min_distance_m = min_distance_m
        self.min_ttc_s = min_ttc_s

    def is_safe(self, candidate: PathCandidate) -> bool:
        if candidate.keepout_violation:
            return False
        if candidate.min_distance_m < self.min_distance_m:
            return False
        if candidate.ttc_s is not None and candidate.ttc_s < self.min_ttc_s:
            return False
        return True
