#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f install/setup.bash ]; then source install/setup.bash; fi

ok=true
for _ in $(seq 1 8); do
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || ok=false
  timeout 2 ros2 topic pub --once /waver/manual_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  sleep 0.05
done

for pat in nav2_container controller_server planner_server bt_navigator waver_base_driver_node serial_cmd_vel_bridge waver_cmd_vel_serial_bridge; do
  pkill -f "${pat}" 2>/dev/null || true
done

if [ "${ok}" = "true" ]; then
  echo "STOP_CONFIRMED"
else
  echo "STOP_FAILED"
  exit 1
fi
