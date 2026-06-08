#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT=${WORKSPACE_ROOT:-/home/chotaehyun/ros2_ws3/FSD_Vehicle}
OUTPUT_ROOT=${OUTPUT_ROOT:-$WORKSPACE_ROOT/experiment_results/gazebo_spatial_response}
TIMEOUT_SEC=${TIMEOUT_SEC:-180}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
RANDOM_SEED=${RANDOM_SEED:-530}

cd "$WORKSPACE_ROOT"
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  ugv_description ugv_gazebo ugv_tools \
  waver_patrol waver_seo_tracking waver_experiment_logger \
  livox_ros_driver2 ros2_livox_simulation

source install/setup.bash

if [ ! -f install/ros2_livox_simulation/lib/libros2_livox.so ]; then
  echo "ERROR: libros2_livox.so not found. Build ros2_livox_simulation first." >&2
  exit 2
fi

export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
unset RMW_IMPLEMENTATION
mkdir -p "$OUTPUT_ROOT"

TRIAL_ID="spatial_lidar_$(date +%Y%m%d_%H%M%S)_seed${RANDOM_SEED}"

timeout --foreground "${TIMEOUT_SEC}s" ros2 launch ugv_gazebo ugv_gazebo_bird_patrol_seo.launch.py \
  use_sim_time:=true \
  use_gui:=false \
  start_remote_panel:=true \
  remote_panel_demo_script:=airport_patrol_trial \
  detector_mode:=lidar \
  classifier_mode:=fake_gazebo \
  enable_fake_bird_classifier:=true \
  enable_trial_logger:=true \
  enable_dataset_logger:=true \
  enable_spatial_response_logger:=true \
  enable_rosbag_record:=false \
  save_images:=false \
  write_coco:=false \
  write_yolo:=false \
  enable_bird_removal_after_detection:=true \
  require_sound_done_for_removal:=true \
  bird_removal_goal_count:=2 \
  active_birds:=bird_1,bird_2,bird_3,bird_4 \
  random_seed:="$RANDOM_SEED" \
  trial_id:="$TRIAL_ID" \
  output_root:="$OUTPUT_ROOT" || true

RUN_DIR=$(find "$OUTPUT_ROOT" -maxdepth 1 -type d -name "${TRIAL_ID}_*" | sort | tail -1)
if [ -z "$RUN_DIR" ]; then
  echo "ERROR: run_dir not found under $OUTPUT_ROOT" >&2
  exit 3
fi

python3 src/waver_experiment_logger/scripts/compute_spatial_response_metrics.py "$RUN_DIR"
echo "RUN_DIR=$RUN_DIR"
