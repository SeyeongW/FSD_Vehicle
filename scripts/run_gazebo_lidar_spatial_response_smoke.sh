#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT=${WORKSPACE_ROOT:-/home/chotaehyun/ugv_ws/FSD_Vehicle}
OUTPUT_ROOT=${OUTPUT_ROOT:-$WORKSPACE_ROOT/experiment_results/gazebo_spatial_response}
TIMEOUT_SEC=${TIMEOUT_SEC:-180}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-30}
RANDOM_SEED=${RANDOM_SEED:-530}
BATCH_TIME="$(date +%Y%m%d_%H%M%S)"
TRIAL_ID="spatial_lidar_${BATCH_TIME}_seed${RANDOM_SEED}"
RUN_ID="${TRIAL_ID}_${BATCH_TIME}"
EXPECTED_RUN_DIR="${OUTPUT_ROOT}/${RUN_ID}"

cd "$WORKSPACE_ROOT"
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --symlink-install --packages-select \
  ugv_description ugv_gazebo ugv_tools \
  waver_patrol waver_seo_tracking waver_experiment_logger \
  livox_ros_driver2 ros2_livox_simulation

set +u
source install/setup.bash
set -u

if [ ! -f install/ros2_livox_simulation/lib/libros2_livox.so ]; then
  echo "ERROR: libros2_livox.so not found. Build ros2_livox_simulation first." >&2
  exit 2
fi

export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
unset RMW_IMPLEMENTATION
mkdir -p "$OUTPUT_ROOT"
mkdir -p "$EXPECTED_RUN_DIR/logs"

cleanup_gazebo() {
  pkill -f "gzserver.*ugv_world.world" >/dev/null 2>&1 || true
  pkill -f "gzclient" >/dev/null 2>&1 || true
}
trap cleanup_gazebo EXIT

(
  set +e
  for attempt in $(seq 1 45); do
    sleep 2
    ros2 topic info -v /cmd_vel > "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info.tmp" 2>&1
    cp "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info.tmp" "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info_last.txt"
    if grep -q "Publisher count: 1" "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info.tmp" \
      && grep -q "safety_cmd_mux_node" "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info.tmp"; then
      mv "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info.tmp" "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info.txt"
      break
    fi
    if [ "$attempt" -eq 45 ]; then
      mv "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info.tmp" "$EXPECTED_RUN_DIR/logs/cmd_vel_topic_info.txt"
    fi
  done
  ros2 topic list > "$EXPECTED_RUN_DIR/logs/topic_list_snapshot.txt" 2>&1
  ros2 node list > "$EXPECTED_RUN_DIR/logs/node_list_snapshot.txt" 2>&1
) &
TOPIC_MONITOR_PID=$!

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
  run_id:="$RUN_ID" \
  run_dir:="$EXPECTED_RUN_DIR" \
  output_root:="$OUTPUT_ROOT" || true

wait "$TOPIC_MONITOR_PID" 2>/dev/null || true

RUN_DIR="$EXPECTED_RUN_DIR"
if [ -z "$RUN_DIR" ]; then
  echo "ERROR: run_dir not found under $OUTPUT_ROOT" >&2
  exit 3
fi

python3 src/waver_experiment_logger/scripts/compute_spatial_response_metrics.py "$RUN_DIR"
python3 scripts/verify_gazebo_spatial_response_trial.py "$RUN_DIR" \
  --mode full \
  --min-removed-birds 2 \
  --require-mid-patrol-preempt
echo "RUN_DIR=$RUN_DIR"
