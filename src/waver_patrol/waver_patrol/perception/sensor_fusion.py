from __future__ import annotations


def degraded_mode(lidar_ok: bool, camera_ok: bool) -> str:
    if lidar_ok:
        return "LIDAR_SAFETY_ONLY" if not camera_ok else "FULL"
    return "STOP"
