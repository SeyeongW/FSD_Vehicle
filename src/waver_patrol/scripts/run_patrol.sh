#!/usr/bin/env bash
set -euo pipefail
cd "${WAVER_WS:-$HOME/ros2_ws}"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch waver_patrol waver_patrol.launch.py waypoints:=src/waver_patrol/waypoints/patrol_outdoor_demo.yaml "$@"
