#!/usr/bin/env bash
set -euo pipefail

TRIALS="${1:-3}"
DETECTOR_MODE="${DETECTOR_MODE:-lidar}"
CLASSIFIER_MODE="${CLASSIFIER_MODE:-fake_gazebo}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$HOME/ugv_ws/FSD_Vehicle/experiments_result/gazebo_bird_patrol}"

cd "$HOME/ugv_ws/FSD_Vehicle"
source /opt/ros/humble/setup.bash
source install/setup.bash

for i in $(seq 1 "$TRIALS"); do
  trial_id="paper_trial_${i}"
  echo "[trial] $trial_id detector_mode=$DETECTOR_MODE"
  timeout 90s ros2 launch ugv_gazebo ugv_gazebo_bird_patrol_seo.launch.py \
    use_gui:=true \
    start_remote_panel:=true \
    remote_panel_demo_script:=start_patrol_once \
    detector_mode:="$DETECTOR_MODE" \
    classifier_mode:="$CLASSIFIER_MODE" \
    trial_id:="$trial_id" \
    output_root:="$OUTPUT_ROOT" \
    enable_dataset_logger:=true \
    enable_trial_logger:=true \
    save_images:=true || true
  latest=$(ls -td "$OUTPUT_ROOT"/"${trial_id}"_* 2>/dev/null | head -1)
  if [[ -z "${latest:-}" ]]; then
    echo "No run directory found for $trial_id" >&2
    exit 1
  fi
  python3 src/waver_experiment_logger/scripts/compute_paper_metrics.py "$latest"
  python3 src/waver_experiment_logger/scripts/check_dataset_integrity.py "$latest" --min-images 1
  python3 src/waver_experiment_logger/scripts/plot_paper_figures.py "$latest"
done
