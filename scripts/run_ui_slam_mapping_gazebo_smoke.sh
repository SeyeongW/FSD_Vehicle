#!/usr/bin/env bash
set -euo pipefail

cd "${WAVER_ROS2_WS5_ROOT:-$HOME/ros2_ws5/FSD_Vehicle}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
if [ -n "${WAVER_RMW_IMPLEMENTATION:-}" ]; then
  export RMW_IMPLEMENTATION="$WAVER_RMW_IMPLEMENTATION"
fi

mkdir -p maps log/ui_slam

cleanup() {
  set +e
  bash scripts/waver_ui_slam_cleanup.sh >/dev/null 2>&1 || true
}

spawn_test_obstacle() {
  local sdf="/tmp/waver_slam_test_box.sdf"
  local log="log/ui_slam/mapping_smoke_obstacle_$(date +%Y%m%d_%H%M%S).log"
  local size_x="${WAVER_TEST_OBSTACLE_SIZE_X:-0.55}"
  local size_y="${WAVER_TEST_OBSTACLE_SIZE_Y:-0.55}"
  local size_z="${WAVER_TEST_OBSTACLE_SIZE_Z:-0.90}"
  local spawn_z="${WAVER_TEST_OBSTACLE_Z:-0.45}"
  cat >"$sdf" <<'EOF'
<sdf version="1.6">
  <model name="slam_test_box">
    <static>true</static>
    <link name="link">
      <pose>0 0 0 0 0 0</pose>
      <collision name="collision"><geometry><box><size>__SIZE_X__ __SIZE_Y__ __SIZE_Z__</size></box></geometry></collision>
      <visual name="visual">
        <geometry><box><size>__SIZE_X__ __SIZE_Y__ __SIZE_Z__</size></box></geometry>
        <material><ambient>0.9 0.1 0.1 1</ambient><diffuse>0.9 0.1 0.1 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
EOF
  sed -i \
    -e "s/__SIZE_X__/${size_x}/g" \
    -e "s/__SIZE_Y__/${size_y}/g" \
    -e "s/__SIZE_Z__/${size_z}/g" \
    "$sdf"
  sleep "${WAVER_TEST_OBSTACLE_DELAY_SEC:-13}"
  ros2 run gazebo_ros spawn_entity.py \
    -entity "${WAVER_TEST_OBSTACLE_NAME:-slam_test_box}" \
    -file "$sdf" \
    -x "${WAVER_TEST_OBSTACLE_X:-1.35}" \
    -y "${WAVER_TEST_OBSTACLE_Y:-0.65}" \
    -z "$spawn_z" \
    >"$log" 2>&1 || true
}

cleanup
trap cleanup EXIT

set +e
timeout --foreground "${WAVER_UI_SLAM_TIMEOUT:-180}" \
  ros2 launch ugv_gazebo ugv_gazebo_ui_slam_mapping.launch.py \
    use_sim_time:=true \
    use_gui:="${WAVER_USE_GUI:-false}" \
    mapping_backend:="${WAVER_MAPPING_BACKEND:-scan_mapper}" \
    start_rviz:="${WAVER_START_RVIZ:-false}" \
    spawn_static_obstacle:="${WAVER_SPAWN_TEST_OBSTACLE:-false}" \
    static_obstacle_x:="${WAVER_TEST_OBSTACLE_X:-1.35}" \
    static_obstacle_y:="${WAVER_TEST_OBSTACLE_Y:-0.65}" \
    static_obstacle_z:="${WAVER_TEST_OBSTACLE_Z:-0.0}" \
    start_remote_panel:=true \
    remote_panel_demo_script:="${WAVER_REMOTE_PANEL_DEMO:-mapping_workflow_smoke}" \
    demo_close_on_finish:=true \
    save_dir:="$HOME/ros2_ws5/FSD_Vehicle/maps" \
  2>&1 | tee "log/ui_slam/mapping_smoke_$(date +%Y%m%d_%H%M%S).log"
status=${PIPESTATUS[0]}
set -e
if [ "$status" -ne 0 ] && [ "$status" -ne 124 ]; then
  exit "$status"
fi

CHECK_ARGS=(
  --map-yaml "$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml"
  --skip-graph
)
if [ "${WAVER_SPAWN_TEST_OBSTACLE:-false}" = "true" ]; then
  CHECK_ARGS+=(
    --expect-obstacle
    --obstacle-x "${WAVER_TEST_OBSTACLE_X:-1.35}"
    --obstacle-y "${WAVER_TEST_OBSTACLE_Y:-0.65}"
    --obstacle-radius-m "${WAVER_TEST_OBSTACLE_CHECK_RADIUS_M:-0.8}"
    --min-obstacle-occupied "${WAVER_TEST_OBSTACLE_MIN_OCCUPIED:-10}"
  )
fi
python3 scripts/check_remote_ui_slam_mapping_result.py "${CHECK_ARGS[@]}"
