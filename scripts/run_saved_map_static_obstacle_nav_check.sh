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

mkdir -p log/ui_slam

cleanup() {
  set +e
  bash scripts/waver_ui_slam_cleanup.sh >/dev/null 2>&1 || true
}

cleanup
trap cleanup EXIT

map_yaml="${WAVER_SAVED_MAP_YAML:-$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml}"
obstacle_x="${WAVER_STATIC_OBSTACLE_X:-1.00}"
obstacle_y="${WAVER_STATIC_OBSTACLE_Y:-0.0}"
goal_x="${WAVER_NAV_GOAL_X:-1.80}"
goal_y="${WAVER_NAV_GOAL_Y:-0.0}"

launch_log="log/ui_slam/static_obstacle_nav_$(date +%Y%m%d_%H%M%S).log"
ros2 launch ugv_gazebo ugv_gazebo_saved_map_nav2.launch.py \
  use_sim_time:=true \
  use_gui:="${WAVER_USE_GUI:-false}" \
  start_remote_panel:=true \
  map:="$map_yaml" \
  spawn_static_obstacle:=true \
  static_obstacle_x:="$obstacle_x" \
  static_obstacle_y:="$obstacle_y" \
  demo_close_on_finish:=false \
  >"$launch_log" 2>&1 &
launch_pid=$!

sleep "${WAVER_NAV_STARTUP_WAIT_SEC:-34}"

python3 scripts/check_saved_map_ui_startup.py --timeout 20
python3 scripts/run_saved_map_nav2_action_check.py \
  --goal-x "$goal_x" \
  --goal-y "$goal_y" \
  --min-odom-distance "${WAVER_MIN_ODOM_DISTANCE:-1.00}" \
  --result-timeout "${WAVER_NAV_RESULT_TIMEOUT:-120}" \
  --require-obstacle-clearance \
  --obstacle-x "$obstacle_x" \
  --obstacle-y "$obstacle_y" \
  --min-obstacle-distance "${WAVER_MIN_OBSTACLE_DISTANCE:-0.42}" \
  --min-lateral-deviation "${WAVER_MIN_LATERAL_DEVIATION:-0.15}"

kill "$launch_pid" >/dev/null 2>&1 || true
wait "$launch_pid" >/dev/null 2>&1 || true
