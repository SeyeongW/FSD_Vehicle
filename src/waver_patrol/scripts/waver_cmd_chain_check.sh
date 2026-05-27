#!/usr/bin/env bash
set -euo pipefail

echo "== Waver command-chain audit =="
ros2 topic info -v /cmd_vel > /tmp/waver_cmd_vel_info.txt 2>/dev/null || {
  echo "ERROR: /cmd_vel is not visible"
  exit 1
}
cat /tmp/waver_cmd_vel_info.txt
PUB_COUNT="$(grep -c 'Endpoint type: PUBLISHER' /tmp/waver_cmd_vel_info.txt || true)"
if [ "$PUB_COUNT" -ne 1 ]; then
  echo "ERROR: expected exactly one /cmd_vel publisher, got $PUB_COUNT"
  exit 1
fi
if grep -E 'controller_server|waver_remote_panel|teleop|ugv_driver|pointcloud|target_goal' /tmp/waver_cmd_vel_info.txt; then
  echo "ERROR: direct /cmd_vel publisher violation detected"
  exit 2
fi
if ! grep -E 'safety_cmd_mux_node|collision_monitor' /tmp/waver_cmd_vel_info.txt >/dev/null; then
  echo "ERROR: final /cmd_vel publisher is not the selected safety gate"
  exit 3
fi

echo
echo "== Candidate command topics =="
ros2 topic info -v /waver/manual_cmd_vel 2>/dev/null || true
ros2 topic info -v /waver/cmd_vel_nav2 2>/dev/null || true
ros2 topic info -v /waver/cmd_vel_nav2_smooth 2>/dev/null || true

echo
echo "PASS: final /cmd_vel has one safety publisher"
