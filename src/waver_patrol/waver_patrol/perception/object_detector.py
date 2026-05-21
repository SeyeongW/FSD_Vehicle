from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DetectedObject:
    label: str
    confidence: float
    distance_m: float | None = None
    dynamic: bool = True


class ObjectDetector:
    def __init__(self, enabled: bool = False):
        self.enabled = enabled

    def detect(self, _frame: object) -> list[DetectedObject]:
        return []
