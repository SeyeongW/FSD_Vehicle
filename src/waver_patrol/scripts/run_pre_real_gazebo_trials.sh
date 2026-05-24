#!/usr/bin/env bash
set -eo pipefail

# Pre-real validation runner.
# 역할:
#   - 실차 투입 전 Gazebo 검증을 height-based H1/H2/H3 기준으로 반복한다.
#   - 반복 안정성을 위해 다중 bird_manager/swarm stress test 대신
#     ugv_world.world + ugv_rover + 단일 bird_test_target + fake cluster interface를 사용한다.
#   - fake cluster publisher는 실제 cluster_node.py와 같은 `/waver/lidar_objects` 인터페이스를 낸다.
#   - serial bridge, real sound output, direct motor control은 켜지지 않는다.

WS="${WAVER_WS:-$HOME/ros2_ws}"
TRIALS="${TRIALS:-1 2 3}"
REQUIRED_SUCCESSES="${REQUIRED_SUCCESSES:-3}"
MAX_WAIT_SEC="${MAX_WAIT_SEC:-150}"
USE_GUI="${USE_GUI:-false}"
RECORD_BAG="${RECORD_BAG:-false}"
TARGET_MIN_HEIGHT_M="${TARGET_MIN_HEIGHT_M:-3.0}"
MIN_DYNAMIC_MOTION_M="${MIN_DYNAMIC_MOTION_M:-0.2}"
MIN_DYNAMIC_VELOCITY_MPS="${MIN_DYNAMIC_VELOCITY_MPS:-0.05}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$WS/experiments_result}"
LOG_DIR="${LOG_DIR:-$OUTPUT_ROOT/run_logs}"
POST_SUCCESS_GRACE_SEC="${POST_SUCCESS_GRACE_SEC:-3}"
GAZEBO_CLEANUP_SLEEP_SEC="${GAZEBO_CLEANUP_SLEEP_SEC:-12}"
ROS_DOMAIN_BASE="${ROS_DOMAIN_BASE:-70}"
RUN_MODE="${RUN_MODE:-isolated_sequences}"
SEQUENCE_DURATION_SEC="${SEQUENCE_DURATION_SEC:-8.0}"
TARGET_Z="${TARGET_Z:-3.2}"

mkdir -p "$LOG_DIR"
cd "$WS"

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

cleanup_gazebo() {
  killall -q gzserver gzclient 2>/dev/null || true
  sleep "$GAZEBO_CLEANUP_SLEEP_SEC"
}

latest_summary_for_trial() {
  local trial_id="$1"
  find "$OUTPUT_ROOT" -maxdepth 2 -path "*/experiment_summary.csv" 2>/dev/null \
    | grep "gazebo_trial_$(printf '%02d' "$trial_id")_" \
    | sort \
    | tail -1
}

summary_success() {
  local summary="$1"
  python3 - "$summary" <<'PY'
import csv
import sys
try:
    with open(sys.argv[1], newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    print(str(bool(rows and str(rows[-1].get("overall_success", "")).lower() == "true")).lower())
except Exception:
    print("false")
PY
}

stop_launch_group() {
  local launch_pid="$1"
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "-$launch_pid" 2>/dev/null || true
    sleep 3
    kill -TERM "-$launch_pid" 2>/dev/null || true
    sleep 2
    kill -KILL "-$launch_pid" 2>/dev/null || true
  fi
  cleanup_gazebo
}

validate_ros_domain_id() {
  local domain_id="$1"
  local label="$2"
  if ! [[ "$domain_id" =~ ^[0-9]+$ ]] || [ "$domain_id" -gt 232 ]; then
    echo "ERROR: $label ROS_DOMAIN_ID=$domain_id is outside the Fast DDS safe range 0..232." >&2
    echo "Set ROS_DOMAIN_BASE to a lower value, for example: ROS_DOMAIN_BASE=180" >&2
    exit 2
  fi
}

if [ "$RUN_MODE" = "single_session" ]; then
  cleanup_gazebo
  stamp="$(date +%Y%m%d_%H%M%S)"
  run_name="pre_real_height_sequence_$stamp"
  log_path="$LOG_DIR/${run_name}.log"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$ROS_DOMAIN_BASE}"
  validate_ros_domain_id "$ROS_DOMAIN_ID" "single-session"
  echo "== Running single-session pre-real validation: $run_name =="
  echo "   robot: ugv_rover, world: ugv_gazebo/worlds/ugv_world.world"
  setsid bash -lc "export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'; source /opt/ros/humble/setup.bash && source '$WS/install/setup.bash' && ros2 launch waver_patrol gazebo_pre_real_validation_headless.launch.py \
    trial_id:=1 \
    target_min_height_m:='$TARGET_MIN_HEIGHT_M' \
    min_dynamic_motion_m:='$MIN_DYNAMIC_MOTION_M' \
    min_dynamic_velocity_mps:='$MIN_DYNAMIC_VELOCITY_MPS' \
    record_bag:=false \
    output_root:='$OUTPUT_ROOT' \
    enable_cluster_node:=false \
    enable_trial_logger:=false \
    target_z:='$TARGET_Z'" >"$log_path" 2>&1 &
  launch_pid=$!

  sleep 18
  set +e
  python3 "$WS/src/FSD_Vehicle/src/waver_patrol/scripts/run_pre_real_sequence_validator.py" \
    --output-root "$OUTPUT_ROOT" \
    --experiment-name "$run_name" \
    --duration-sec "$SEQUENCE_DURATION_SEC" \
    --target-z "$TARGET_Z"
  validator_rc=$?
  set -e

  stop_launch_group "$launch_pid"
  echo "backend log: $log_path"
  summary="$OUTPUT_ROOT/$run_name/experiment_summary.csv"
  if [ -f "$summary" ]; then
    cat "$summary"
  else
    echo "missing summary: $summary"
    tail -160 "$log_path" || true
  fi
  python3 "$WS/src/FSD_Vehicle/src/waver_patrol/scripts/generate_pre_real_report.py" \
  --results_dir "$OUTPUT_ROOT/$run_name/results" \
    --output "$OUTPUT_ROOT/$run_name/results/final_pass_fail_report.md" || true
  exit "$validator_rc"
fi

success_count=0
total_count=0

for trial_id in $TRIALS; do
  total_count=$((total_count + 1))
  cleanup_gazebo
  stamp="$(date +%Y%m%d_%H%M%S)"
  run_name="pre_real_height_H${trial_id}_$stamp"
  log_path="$LOG_DIR/${run_name}.log"
  trial_domain=$((ROS_DOMAIN_BASE + total_count))
  validate_ros_domain_id "$trial_domain" "trial H$trial_id"
  echo "== Running isolated pre-real Gazebo validation H$trial_id =="

  setsid bash -lc "export ROS_DOMAIN_ID='$trial_domain'; source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch waver_patrol gazebo_pre_real_validation_headless.launch.py \
    trial_id:='$trial_id' \
    target_min_height_m:='$TARGET_MIN_HEIGHT_M' \
    min_dynamic_motion_m:='$MIN_DYNAMIC_MOTION_M' \
    min_dynamic_velocity_mps:='$MIN_DYNAMIC_VELOCITY_MPS' \
    record_bag:='$RECORD_BAG' \
    output_root:='$OUTPUT_ROOT' \
    enable_cluster_node:=false \
    enable_trial_logger:=false \
    target_z:='$TARGET_Z'" >"$log_path" 2>&1 &
  launch_pid=$!

  sleep 18
  set +e
  ROS_DOMAIN_ID="$trial_domain" python3 "$WS/src/FSD_Vehicle/src/waver_patrol/scripts/run_pre_real_sequence_validator.py" \
    --output-root "$OUTPUT_ROOT" \
    --experiment-name "$run_name" \
    --duration-sec "$SEQUENCE_DURATION_SEC" \
    --target-z "$TARGET_Z" \
    --only-trial "$trial_id"
  validator_rc=$?
  set -e

  stop_launch_group "$launch_pid"
  summary="$OUTPUT_ROOT/$run_name/experiment_summary.csv"
  echo "summary: ${summary:-missing}"
  if [ -f "$summary" ]; then
    cat "$summary"
    ok="$(summary_success "$summary")"
  else
    tail -120 "$log_path" || true
  fi

  if [ "$validator_rc" -eq 0 ] && [ "$ok" = "true" ]; then
    success_count=$((success_count + 1))
    echo "H$trial_id: SUCCESS"
  else
    echo "H$trial_id: FAIL"
    tail -120 "$log_path" || true
  fi
done

echo "Pre-real Gazebo validation successful: $success_count / $total_count"
python3 "$WS/src/FSD_Vehicle/src/waver_patrol/scripts/analyze_pre_real_gazebo_trials.py" \
  --input_dir "$OUTPUT_ROOT" \
  --output_dir "$OUTPUT_ROOT/results" || true
python3 "$WS/src/FSD_Vehicle/src/waver_patrol/scripts/plot_pre_real_gazebo_results.py" \
  --results_dir "$OUTPUT_ROOT/results" || true
python3 "$WS/src/FSD_Vehicle/src/waver_patrol/scripts/generate_pre_real_report.py" \
  --results_dir "$OUTPUT_ROOT/results" \
  --output "$OUTPUT_ROOT/results/final_pass_fail_report.md" || true

test "$success_count" -ge "$REQUIRED_SUCCESSES"
