from __future__ import annotations


def lifecycle_nodes() -> list[str]:
    try:
        import subprocess

        result = subprocess.run(["ros2", "node", "list"], text=True, capture_output=True, check=False)
        return result.stdout.splitlines() if result.returncode == 0 else []
    except OSError:
        return []
