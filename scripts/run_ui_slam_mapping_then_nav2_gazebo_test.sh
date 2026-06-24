#!/usr/bin/env bash
set -euo pipefail

cd "${WAVER_ROS2_WS5_ROOT:-$HOME/ros2_ws5/FSD_Vehicle}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
if [ -n "${WAVER_RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION="$WAVER_RMW_IMPLEMENTATION"
fi

mkdir -p maps log/ui_slam

cleanup_all() {
  set +e
  bash scripts/waver_ui_slam_cleanup.sh >/dev/null 2>&1 || true
}

cleanup_all
trap cleanup_all EXIT

echo "[UI_SLAM_NAV2] step 1/2: run UI SLAM mapping and save map"
WAVER_USE_GUI="${WAVER_USE_GUI:-false}" \
WAVER_UI_SLAM_TIMEOUT="${WAVER_UI_SLAM_TIMEOUT:-230}" \
WAVER_REMOTE_PANEL_DEMO="${WAVER_REMOTE_PANEL_DEMO:-mapping_workflow_smoke}" \
  bash scripts/run_ui_slam_mapping_gazebo_smoke.sh

echo "[UI_SLAM_NAV2] cleaning mapping/Gazebo processes before saved-map Nav2"
cleanup_all
sleep 3

echo "[UI_SLAM_NAV2] step 2/2: run saved-map AMCL/Nav2 and send a real NavigateToPose goal"
nav_log="log/ui_slam/saved_map_nav2_$(date +%Y%m%d_%H%M%S).log"
set +e
timeout --foreground "${WAVER_NAV2_TEST_TIMEOUT:-180}" \
  ros2 launch ugv_gazebo ugv_gazebo_saved_map_nav2.launch.py \
    use_sim_time:=true \
    use_gui:="${WAVER_USE_GUI:-false}" \
    start_remote_panel:=false \
    map:="$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml" \
  >"$nav_log" 2>&1 &
launch_pid=$!
set -e

cleanup() {
  set +e
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    sleep 5
  fi
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -TERM "$launch_pid" 2>/dev/null || true
    sleep 2
  fi
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -KILL "$launch_pid" 2>/dev/null || true
  fi
  cleanup_all
}
trap cleanup EXIT

sleep 22
python3 scripts/run_saved_map_nav2_action_check.py \
  --goal-x "${WAVER_NAV2_GOAL_X:-0.45}" \
  --goal-y "${WAVER_NAV2_GOAL_Y:-0.0}" \
  --goal-yaw "${WAVER_NAV2_GOAL_YAW:-0.0}" \
  --min-odom-distance "${WAVER_NAV2_MIN_ODOM_DISTANCE:-0.15}" \
  | tee -a "$nav_log"

echo "[UI_SLAM_NAV2] log=$nav_log"
