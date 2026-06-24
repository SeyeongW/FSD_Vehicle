#!/usr/bin/env bash
set -euo pipefail

ROOT="${WAVER_UGV_WS_ROOT:-$HOME/ugv_ws/FSD_Vehicle}"

term_pattern() {
  local pattern="$1"
  local pids
  pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
  [ -z "$pids" ] && return 0
  while read -r pid; do
    [ -z "$pid" ] && continue
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "${BASHPID:-$$}" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill -TERM "$pid" 2>/dev/null || true
  done <<< "$pids"
}

kill_pattern() {
  local pattern="$1"
  local pids
  pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
  [ -z "$pids" ] && return 0
  while read -r pid; do
    [ -z "$pid" ] && continue
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "${BASHPID:-$$}" ] && continue
    [ "$pid" = "$PPID" ] && continue
    kill -KILL "$pid" 2>/dev/null || true
  done <<< "$pids"
}

cleanup_once() {
  local signal="${1:-TERM}"
  local matcher="term_pattern"
  if [ "$signal" = "KILL" ]; then
    matcher="kill_pattern"
  fi

  "$matcher" "ros2 launch ugv_gazebo ugv_gazebo_ui_slam_mapping.launch.py"
  "$matcher" "ros2 launch ugv_gazebo ugv_gazebo_saved_map_nav2.launch.py"
  "$matcher" "gzserver --verbose .*ugv_world.world"
  "$matcher" "gzclient"
  "$matcher" "rviz2.*rviz2_ui_slam"

  "$matcher" "$ROOT/install/waver_patrol/lib/waver_patrol/scan_republisher_node"
  "$matcher" "$ROOT/install/waver_patrol/lib/waver_patrol/mapping_workflow_manager_node"
  "$matcher" "$ROOT/install/waver_patrol/lib/waver_patrol/mapping_backend_manager_node"
  "$matcher" "$ROOT/install/waver_patrol/lib/waver_patrol/static_map_state_publisher_node"
  "$matcher" "$ROOT/install/waver_patrol/lib/waver_patrol/laser_scan_occupancy_mapper_node"
  "$matcher" "$ROOT/install/waver_patrol/lib/waver_patrol/mission_patrol_manager_node"
  "$matcher" "$ROOT/install/waver_patrol/lib/waver_patrol/safety_cmd_mux_node"
  "$matcher" "$ROOT/install/waver_patrol/lib/waver_patrol/simple_nav2_cmd_sim_node"
  "$matcher" "$ROOT/install/ugv_tools/lib/ugv_tools/waver_gazebo_patrol"
  "$matcher" "$ROOT/install/ugv_tools/lib/ugv_tools/waver_remote_panel"

  "$matcher" "/opt/ros/humble/lib/slam_toolbox/async_slam_toolbox_node"
  "$matcher" "/opt/ros/humble/lib/nav2_map_server/map_server"
  "$matcher" "/opt/ros/humble/lib/nav2_amcl/amcl"
  "$matcher" "/opt/ros/humble/lib/nav2_controller/controller_server"
  "$matcher" "/opt/ros/humble/lib/nav2_planner/planner_server"
  "$matcher" "/opt/ros/humble/lib/nav2_behaviors/behavior_server"
  "$matcher" "/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator"
  "$matcher" "/opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager"
  "$matcher" "/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher"
}

cleanup_once TERM
sleep "${WAVER_CLEANUP_TERM_WAIT_SEC:-3}"
cleanup_once KILL

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
  sleep 1
  ros2 daemon start >/dev/null 2>&1 || true
fi

echo "[WAVER_UI_SLAM_CLEANUP] done"
