from __future__ import annotations

import math
import statistics
from collections import deque


class LidarAnomalyFilter:
    def __init__(self, window: int = 3):
        self.history: deque[list[float]] = deque(maxlen=window)

    def filter_ranges(self, ranges: list[float], range_min: float = 0.05, range_max: float = 12.0) -> list[float]:
        clean = [value if math.isfinite(value) and range_min <= value <= range_max else math.inf for value in ranges]
        self.history.append(clean)
        if len(self.history) < 2:
            return clean
        output: list[float] = []
        for values in zip(*self.history):
            finite = [value for value in values if math.isfinite(value)]
            output.append(statistics.median(finite) if finite else math.inf)
        return output
