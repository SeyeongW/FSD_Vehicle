from __future__ import annotations

import math
import time
from typing import Any


def monotonic() -> float:
    return time.monotonic()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def finite_float(value: Any, name: str = "value") -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite float") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def yaw_deg_to_quat(yaw_deg: float) -> tuple[float, float, float, float]:
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def sign(value: float, deadband: float = 1e-9) -> int:
    if value > deadband:
        return 1
    if value < -deadband:
        return -1
    return 0
