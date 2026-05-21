from __future__ import annotations


def gps_patrol_enabled(has_gps: bool, has_rtk: bool) -> bool:
    return has_gps and has_rtk
