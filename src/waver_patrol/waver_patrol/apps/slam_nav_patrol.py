from __future__ import annotations

import subprocess


def main() -> None:
    subprocess.run(["ros2", "launch", "waver_patrol", "waver_slam_nav_patrol.launch.py"], check=False)
