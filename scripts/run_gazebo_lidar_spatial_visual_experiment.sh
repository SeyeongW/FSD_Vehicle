#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT=${WORKSPACE_ROOT:-/home/chotaehyun/ros2_ws3/FSD_Vehicle}
OUTPUT_ROOT=${OUTPUT_ROOT:-$WORKSPACE_ROOT/experiment_results/gazebo_bird_patrol}
TIMEOUT_SEC=${TIMEOUT_SEC:-260}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
RANDOM_SEED=${RANDOM_SEED:-530}
USE_GUI=${USE_GUI:-false}
ENABLE_RVIZ=${ENABLE_RVIZ:-false}
START_REMOTE_PANEL=${START_REMOTE_PANEL:-true}
ENABLE_ROSBAG=${ENABLE_ROSBAG:-false}
SKIP_BUILD=${SKIP_BUILD:-false}
TRIAL_ID=${TRIAL_ID:-codex_spatial_lidar_visual}
ACTIVE_BIRDS=${ACTIVE_BIRDS:-bird_1,bird_2}
BIRD_REMOVAL_GOAL_COUNT=${BIRD_REMOVAL_GOAL_COUNT:-2}

cd "$WORKSPACE_ROOT"
set +u
source /opt/ros/humble/setup.bash
set -u

if [ "$SKIP_BUILD" != "true" ]; then
  colcon build --symlink-install --packages-select \
    ugv_description ugv_gazebo ugv_tools \
    waver_patrol waver_seo_tracking waver_experiment_logger \
    livox_ros_driver2 ros2_livox_simulation
fi

set +u
source install/setup.bash
set -u

export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
unset RMW_IMPLEMENTATION
mkdir -p "$OUTPUT_ROOT"

cleanup_gazebo() {
  pkill -f "gzserver.*ugv_world.world" >/dev/null 2>&1 || true
  pkill -f "gzclient" >/dev/null 2>&1 || true
}
trap cleanup_gazebo EXIT
cleanup_gazebo

RUN_TRIAL_ID="${TRIAL_ID}_$(date +%Y%m%d_%H%M%S)_seed${RANDOM_SEED}"

set +e
timeout --foreground "${TIMEOUT_SEC}s" ros2 launch ugv_gazebo ugv_gazebo_bird_patrol_seo.launch.py \
  use_sim_time:=true \
  use_gui:="$USE_GUI" \
  enable_rviz:="$ENABLE_RVIZ" \
  start_remote_panel:="$START_REMOTE_PANEL" \
  remote_panel_demo_script:=airport_patrol_trial \
  detector_mode:=lidar \
  classifier_mode:=fake_gazebo \
  enable_fake_bird_classifier:=true \
  enable_trial_logger:=true \
  enable_dataset_logger:=true \
  enable_spatial_response_logger:=true \
  enable_spatial_debug_viz:=true \
  enable_rosbag_record:="$ENABLE_ROSBAG" \
  save_images:=false \
  write_coco:=false \
  write_yolo:=false \
  enable_bird_removal_after_detection:=true \
  require_sound_done_for_removal:=true \
  bird_removal_goal_count:="$BIRD_REMOVAL_GOAL_COUNT" \
  active_birds:="$ACTIVE_BIRDS" \
  random_seed:="$RANDOM_SEED" \
  trial_id:="$RUN_TRIAL_ID" \
  output_root:="$OUTPUT_ROOT"
LAUNCH_STATUS=$?
set -e

RUN_DIR=$(find "$OUTPUT_ROOT" -maxdepth 1 -type d -name "${RUN_TRIAL_ID}_*" | sort | tail -1)
if [ -z "$RUN_DIR" ]; then
  echo "ERROR: run_dir not found under $OUTPUT_ROOT for trial $RUN_TRIAL_ID" >&2
  exit 3
fi

python3 src/waver_experiment_logger/scripts/compute_spatial_response_metrics.py "$RUN_DIR"
python3 src/waver_experiment_logger/scripts/plot_spatial_response_figures.py "$RUN_DIR"
python3 scripts/verify_gazebo_spatial_response_trial.py "$RUN_DIR" --min-removed-birds "$BIRD_REMOVAL_GOAL_COUNT"

echo "RUN_DIR=$RUN_DIR"
echo "LAUNCH_STATUS=$LAUNCH_STATUS"
echo "GAZEBO_LIDAR_SPATIAL_VISUAL_EXPERIMENT=PASS"
