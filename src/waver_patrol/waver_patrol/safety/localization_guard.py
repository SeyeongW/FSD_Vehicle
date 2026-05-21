from __future__ import annotations

import math
from dataclasses import dataclass, field

from waver_patrol.safety.command import SafetyEvent
from waver_patrol.utils import monotonic


@dataclass(frozen=True)
class LocalizationDecision:
    action: str
    covariance_xy: float
    events: list[SafetyEvent] = field(default_factory=list)


@dataclass(frozen=True)
class LocalizationConfig:
    covariance_warn_xy: float = 0.5
    covariance_stop_xy: float = 1.5
    pose_stale_s: float = 1.0
    tf_stale_s: float = 1.0


class LocalizationGuard:
    def __init__(self, config: LocalizationConfig | None = None):
        self.config = config or LocalizationConfig()

    def evaluate(
        self,
        covariance: list[float] | tuple[float, ...] | None,
        *,
        pose_stamp: float | None = None,
        tf_stamp: float | None = None,
        now: float | None = None,
    ) -> LocalizationDecision:
        current_time = monotonic() if now is None else now
        events: list[SafetyEvent] = []
        if pose_stamp is None or current_time - pose_stamp > self.config.pose_stale_s:
            events.append(SafetyEvent("critical", "pose_stale", "Pose is stale or missing"))
            return LocalizationDecision("STOP", math.inf, events)
        if tf_stamp is None or current_time - tf_stamp > self.config.tf_stale_s:
            events.append(SafetyEvent("critical", "tf_stale", "TF is stale or missing"))
            return LocalizationDecision("STOP", math.inf, events)
        if covariance is None or len(covariance) < 8:
            events.append(SafetyEvent("critical", "covariance_missing", "Pose covariance missing"))
            return LocalizationDecision("STOP", math.inf, events)
        covariance_xy = math.sqrt(max(float(covariance[0]), 0.0) + max(float(covariance[7]), 0.0))
        if covariance_xy >= self.config.covariance_stop_xy:
            events.append(SafetyEvent("critical", "pose_covariance_stop", "Pose covariance above stop threshold"))
            return LocalizationDecision("STOP", covariance_xy, events)
        if covariance_xy >= self.config.covariance_warn_xy:
            events.append(SafetyEvent("warning", "pose_covariance_warn", "Pose covariance above warning threshold"))
            return LocalizationDecision("SLOW_DOWN", covariance_xy, events)
        return LocalizationDecision("OK", covariance_xy, events)
