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

set +e
timeout --foreground "${WAVER_UI_SLAM_PATROL_TIMEOUT:-330}" \
  ros2 launch ugv_gazebo ugv_gazebo_ui_slam_mapping.launch.py \
    use_sim_time:=true \
    use_gui:="${WAVER_USE_GUI:-true}" \
    mapping_backend:="${WAVER_MAPPING_BACKEND:-slam_toolbox}" \
    start_remote_panel:=true \
    remote_panel_demo_script:="${WAVER_REMOTE_PANEL_DEMO:-mapping_full_coverage}" \
    demo_close_on_finish:=true \
    save_dir:="$HOME/ros2_ws5/FSD_Vehicle/maps" \
  2>&1 | tee "log/ui_slam/mapping_then_patrol_$(date +%Y%m%d_%H%M%S).log"
status=${PIPESTATUS[0]}
set -e
if [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; then
  exit "$status"
fi

python3 scripts/check_remote_ui_slam_mapping_result.py \
  --map-yaml "$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml" \
  --skip-graph
