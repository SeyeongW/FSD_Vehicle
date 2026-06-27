#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT=${WORKSPACE_ROOT:-$HOME/ros2_ws5/FSD_Vehicle}
OUTPUT_ROOT=${OUTPUT_ROOT:-$WORKSPACE_ROOT/experiment_results/gazebo_bird_patrol}
TIMEOUT_SEC=${TIMEOUT_SEC:-260}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
RANDOM_SEED=${RANDOM_SEED:-530}
USE_GUI=${USE_GUI:-false}
ENABLE_RVIZ=${ENABLE_RVIZ:-false}
START_REMOTE_PANEL=${START_REMOTE_PANEL:-true}
ENABLE_ROSBAG=${ENABLE_ROSBAG:-false}
SKIP_BUILD=${SKIP_BUILD:-false}
TRIAL_ID=${TRIAL_ID:-codex_spatial_lidar_visual}
ACTIVE_BIRDS=${ACTIVE_BIRDS:-bird_1,bird_2}
BIRD_REMOVAL_GOAL_COUNT=${BIRD_REMOVAL_GOAL_COUNT:-2}
BATCH_TIME="$(date +%Y%m%d_%H%M%S)"
RUN_TRIAL_ID="${TRIAL_ID:-spatial_lidar_${BATCH_TIME}_seed${RANDOM_SEED}}"
RUN_ID="${RUN_TRIAL_ID}_${BATCH_TIME}"
EXPECTED_RUN_DIR="${OUTPUT_ROOT}/${RUN_ID}"

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
mkdir -p "$EXPECTED_RUN_DIR/logs"

cleanup_gazebo() {
  pkill -f "gzserver.*ugv_world.world" >/dev/null 2>&1 || true
  pkill -f "gzclient" >/dev/null 2>&1 || true
}
trap cleanup_gazebo EXIT
cleanup_gazebo

(
  set +e
  mkdir -p "$EXPECTED_RUN_DIR/logs"
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
  run_id:="$RUN_ID" \
  run_dir:="$EXPECTED_RUN_DIR" \
  output_root:="$OUTPUT_ROOT"
LAUNCH_STATUS=$?
set -e

wait "$TOPIC_MONITOR_PID" 2>/dev/null || true

if [ "$LAUNCH_STATUS" -ne 0 ] && [ "$LAUNCH_STATUS" -ne 124 ]; then
  echo "ERROR: ros2 launch failed with status=$LAUNCH_STATUS" >&2
  exit "$LAUNCH_STATUS"
fi

# Give ROS nodes a short chance to finish shutdown hooks before post-processing.
# Some loggers write summary metrics on exit; the offline metrics below must be
# the final authority used by the verifier.
sleep 2

RUN_DIR="$EXPECTED_RUN_DIR"
if [ ! -d "$RUN_DIR" ]; then
  echo "ERROR: expected run_dir not found: $RUN_DIR" >&2
  exit 3
fi
if [ ! -f "$RUN_DIR/logs/spatial_distance_timeseries.csv" ]; then
  FALLBACK_RUN_DIR=$(find "$OUTPUT_ROOT" -maxdepth 3 -type f \
    -path "*/logs/spatial_distance_timeseries.csv" \
    | sed 's#/logs/spatial_distance_timeseries.csv##' \
    | grep "$RUN_TRIAL_ID" \
    | sort \
    | tail -1)
  if [ -n "$FALLBACK_RUN_DIR" ]; then
    RUN_DIR="$FALLBACK_RUN_DIR"
  else
    echo "ERROR: spatial_distance_timeseries.csv not found in $EXPECTED_RUN_DIR" >&2
    exit 4
  fi
fi

python3 src/waver_experiment_logger/scripts/compute_spatial_response_metrics.py "$RUN_DIR"
python3 src/waver_experiment_logger/scripts/plot_spatial_response_figures.py "$RUN_DIR"
python3 src/waver_experiment_logger/scripts/compute_spatial_response_metrics.py "$RUN_DIR"
python3 scripts/verify_gazebo_spatial_response_trial.py "$RUN_DIR" --mode full --min-removed-birds "$BIRD_REMOVAL_GOAL_COUNT"

echo "RUN_DIR=$RUN_DIR"
echo "LAUNCH_STATUS=$LAUNCH_STATUS"
echo "VERIFY_GAZEBO_SPATIAL_RESPONSE=PASS"
echo "GAZEBO_LIDAR_SPATIAL_VISUAL_EXPERIMENT=PASS"
