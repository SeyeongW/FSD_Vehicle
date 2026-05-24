#!/usr/bin/env bash
set -eo pipefail

# Gazebo mission repeat runner.
# 역할:
#   - 기존 ugv_gazebo/worlds/ugv_world.world, ugv_rover, bird_manager 기반으로
#     Waver target mission trial을 3회 반복한다.
#   - experiment_summary.csv에 overall_success=true가 찍히면 해당 trial launch를 종료하고
#     다음 trial로 넘어간다.
#   - 실차 주행용이 아니며, require_scan=false와 sim goal arrival을 쓰는 Gazebo 검증 전용이다.

WS="${WAVER_WS:-$HOME/ros2_ws}"
TRIALS="${TRIALS:-1 2 3}"
MAX_WAIT_SEC="${MAX_WAIT_SEC:-150}"
USE_GUI="${USE_GUI:-false}"
RECORD_BAG="${RECORD_BAG:-false}"
MIN_MOTION_DISTANCE_M="${MIN_MOTION_DISTANCE_M:-3.0}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$WS/experiments_result}"
LOG_DIR="${LOG_DIR:-$OUTPUT_ROOT/run_logs}"
GAZEBO_SIM_MAX_LINEAR_SPEED="${GAZEBO_SIM_MAX_LINEAR_SPEED:-1.5}"
REQUIRED_SUCCESSES="${REQUIRED_SUCCESSES:-3}"
SPAWN_UGV_BIRD_SWARM="${SPAWN_UGV_BIRD_SWARM:-true}"
POST_SUCCESS_GRACE_SEC="${POST_SUCCESS_GRACE_SEC:-6}"
if [ "$SPAWN_UGV_BIRD_SWARM" = "true" ]; then
  BIRD_MANAGER_ACTIVE_BIRDS="${BIRD_MANAGER_ACTIVE_BIRDS:-bird_single,bird_swarm_1,bird_swarm_2,bird_swarm_3,bird_swarm_4,bird_swarm_5}"
else
  BIRD_MANAGER_ACTIVE_BIRDS="${BIRD_MANAGER_ACTIVE_BIRDS:-bird_single}"
fi

mkdir -p "$LOG_DIR"
cd "$WS"

source /opt/ros/humble/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi

WORLD_FILE="${WORLD_FILE:-$(ros2 pkg prefix ugv_gazebo)/share/ugv_gazebo/worlds/ugv_world.world}"
ROBOT_SDF="${ROBOT_SDF:-$(ros2 pkg prefix ugv_gazebo)/share/ugv_gazebo/models/ugv_rover/model.sdf}"

cleanup_gazebo() {
  killall -q gzserver gzclient 2>/dev/null || true
  sleep 3
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

success_count=0
total_count=0

for trial_id in $TRIALS; do
  total_count=$((total_count + 1))
  cleanup_gazebo
  log_path="$LOG_DIR/trial_$(printf '%02d' "$trial_id")_$(date +%Y%m%d_%H%M%S).log"
  echo "== Running ugv_world bird mission trial $trial_id =="

  setsid bash -lc "source /opt/ros/humble/setup.bash && source install/setup.bash && export BIRD_MANAGER_ACTIVE_BIRDS='$BIRD_MANAGER_ACTIVE_BIRDS' && ros2 launch waver_patrol gazebo_moving_object_trial.launch.py \
    start_gazebo:=true \
    use_gui:=$USE_GUI \
    world_file:='$WORLD_FILE' \
    spawn_robot:=true \
    robot_entity:=ugv_rover \
    robot_sdf_file:='$ROBOT_SDF' \
    spawn_target:=false \
    spawn_ugv_bird_single:=true \
    spawn_ugv_bird_swarm:=$SPAWN_UGV_BIRD_SWARM \
    enable_ugv_bird_manager:=true \
    enable_gazebo_bird_bridge:=true \
    enable_cluster_node:=false \
    enable_fake_camera_classification:=true \
    trial_id:='$trial_id' \
    target_min_height_m:='${TARGET_MIN_HEIGHT_M:-3.0}' \
    min_dynamic_motion_m:='${MIN_DYNAMIC_MOTION_M:-0.2}' \
    min_dynamic_velocity_mps:='${MIN_DYNAMIC_VELOCITY_MPS:-0.05}' \
    target_z:=6.0 \
    require_scan:=false \
    enable_mission_stack:=true \
    enable_simple_nav2_cmd_sim:=true \
    enable_moving_object_motion_filter:=true \
    enable_trial_logger:=true \
    record_bag:='$RECORD_BAG' \
    output_root:='$OUTPUT_ROOT' \
    gazebo_sim_max_linear_speed:='$GAZEBO_SIM_MAX_LINEAR_SPEED'" >"$log_path" 2>&1 &
  launch_pid=$!

  ok="false"
  summary=""
  for _ in $(seq 1 "$MAX_WAIT_SEC"); do
    summary="$(latest_summary_for_trial "$trial_id" || true)"
    if [ -n "$summary" ]; then
      ok="$(summary_success "$summary")"
      if [ "$ok" = "true" ]; then
        break
      fi
    fi
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      break
    fi
    sleep 1
  done

  if [ "$ok" = "true" ] && [ "$POST_SUCCESS_GRACE_SEC" -gt 0 ]; then
    sleep "$POST_SUCCESS_GRACE_SEC"
  fi

  stop_launch_group "$launch_pid"
  summary="$(latest_summary_for_trial "$trial_id" || true)"
  echo "summary: ${summary:-missing}"
  if [ -n "$summary" ]; then
    cat "$summary"
    ok="$(summary_success "$summary")"
  else
    tail -120 "$log_path" || true
  fi

  if [ "$ok" = "true" ]; then
    success_count=$((success_count + 1))
    echo "trial $trial_id: SUCCESS"
  else
    echo "trial $trial_id: FAIL"
    tail -120 "$log_path" || true
  fi
done

echo "Gazebo mission trials successful: $success_count / $total_count"
python3 "$WS/src/FSD_Vehicle/src/waver_patrol/scripts/analyze_gazebo_trials.py" \
  --input_dir "$OUTPUT_ROOT" \
  --output_dir "$OUTPUT_ROOT/results" || true

test "$success_count" -ge "$REQUIRED_SUCCESSES"
