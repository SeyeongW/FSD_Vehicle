#!/usr/bin/env bash
set -euo pipefail

cd "${WAVER_WS3_ROOT:-$HOME/ros2_ws3/FSD_Vehicle}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
if [ -n "${WAVER_RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION="$WAVER_RMW_IMPLEMENTATION"
else
  unset RMW_IMPLEMENTATION || true
fi

mkdir -p maps log/ui_slam

exec ros2 launch ugv_gazebo ugv_gazebo_ui_slam_mapping.launch.py \
  use_sim_time:=true \
  use_gui:="${WAVER_USE_GUI:-true}" \
  mapping_backend:="${WAVER_MAPPING_BACKEND:-slam_toolbox}" \
  start_remote_panel:=true \
  demo_close_on_finish:=false \
  save_dir:="$HOME/ros2_ws3/FSD_Vehicle/maps"
