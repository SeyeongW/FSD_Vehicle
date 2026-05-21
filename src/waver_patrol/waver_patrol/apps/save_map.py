from __future__ import annotations

import subprocess


def main() -> None:
    subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", "maps/waver_map"], check=False)
