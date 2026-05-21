from __future__ import annotations

import subprocess


def lsof_port(port: str) -> list[str]:
    result = subprocess.run(["lsof", port], text=True, capture_output=True, check=False)
    if result.returncode not in (0, 1):
        return []
    return [line for line in result.stdout.splitlines() if line.strip()]


def detect_process_conflicts(patterns: list[str] | None = None) -> list[str]:
    patterns = patterns or ["app.py", "ugv_driver", "bringup", "serial"]
    result = subprocess.run(["ps", "aux"], text=True, capture_output=True, check=False)
    lines = result.stdout.splitlines()
    return [line for line in lines if any(pattern in line for pattern in patterns) and "grep" not in line]
