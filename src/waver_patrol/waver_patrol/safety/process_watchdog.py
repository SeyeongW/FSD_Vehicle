from __future__ import annotations

import subprocess


def pgrep(pattern: str) -> list[str]:
    result = subprocess.run(["pgrep", "-af", pattern], check=False, text=True, capture_output=True)
    if result.returncode not in (0, 1):
        return []
    return [line for line in result.stdout.splitlines() if line.strip()]
