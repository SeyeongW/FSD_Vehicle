#!/usr/bin/env bash
set -euo pipefail
cd "${WAVER_WS:-$HOME/ros2_ws}"
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true
ros2 run waver_patrol stop_robot "$@"
