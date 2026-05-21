from __future__ import annotations


def run_task(task: str) -> str:
    if task in {"snapshot", "health_log", "scan_left_right", "battery_check", "sensor_check"}:
        return f"{task}:queued"
    return f"{task}:unsupported"
