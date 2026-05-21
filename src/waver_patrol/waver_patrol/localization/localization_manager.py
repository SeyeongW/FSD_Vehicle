from __future__ import annotations

from waver_patrol.safety.localization_guard import LocalizationDecision


def patrol_allowed(decision: LocalizationDecision, require_good_pose: bool = True) -> bool:
    if not require_good_pose:
        return decision.action != "STOP"
    return decision.action == "OK"
