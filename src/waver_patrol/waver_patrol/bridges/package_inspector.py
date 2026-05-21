from __future__ import annotations

import subprocess


def ros2_packages(pattern: str = "") -> list[str]:
    cmd = ["ros2", "pkg", "list"]
    result = subprocess.run(cmd, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        return []
    packages = result.stdout.splitlines()
    return [pkg for pkg in packages if pattern in pkg]


def package_exists(name: str) -> bool:
    return name in ros2_packages()
