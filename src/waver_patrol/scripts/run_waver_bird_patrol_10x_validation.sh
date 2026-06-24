#!/usr/bin/env bash
set -euo pipefail

WS="${WS:-$HOME/ugv_ws/FSD_Vehicle}"
TRIALS="${TRIALS:-10}"
BASE_DOMAIN="${ROS_DOMAIN_ID:-50}"
TIMEOUT_SEC="${TIMEOUT_SEC:-420}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$WS/experiment_results/gazebo_bird_patrol}"
SIM_NAV_MAX_LINEAR_SPEED="${SIM_NAV_MAX_LINEAR_SPEED:-0.70}"
SIM_NAV_MAX_ANGULAR_SPEED="${SIM_NAV_MAX_ANGULAR_SPEED:-0.95}"

cd "$WS"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

mkdir -p "$OUTPUT_ROOT"

cleanup_trial() {
  pkill -f "ros2 launch ugv_gazebo ugv_gazebo_bird_patrol_seo.launch.py" 2>/dev/null || true
  pkill -f "waver_bird_patrol_trial_validator.py" 2>/dev/null || true
  pkill -f "gzserver.*ugv_world.world" 2>/dev/null || true
  pkill -f "gzclient" 2>/dev/null || true
  sleep 3
}

trap cleanup_trial EXIT

for trial in $(seq 1 "$TRIALS"); do
  cleanup_trial
  export ROS_DOMAIN_ID=$((BASE_DOMAIN + trial))
  export RMW_IMPLEMENTATION="${WAVER_RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  seed=$(( RANDOM + trial * 1000 ))
  run_id="validation_$(date +%Y%m%d_%H%M%S)_trial_${trial}"
  echo "[TRIAL $trial/$TRIALS] ROS_DOMAIN_ID=$ROS_DOMAIN_ID seed=$seed run_id=$run_id"

  ros2 launch ugv_gazebo ugv_gazebo_bird_patrol_seo.launch.py \
    use_gui:=false \
    enable_rviz:=false \
    start_remote_panel:=false \
    random_seed:="$seed" \
    active_birds:=bird_1,bird_2,bird_3,bird_4,bird_5 \
    bird_stable_demo_spawn:=false \
    bird_release_interval_sec:=7.0 \
    bird_removal_goal_count:=5 \
    sim_nav_max_linear_speed:="$SIM_NAV_MAX_LINEAR_SPEED" \
    sim_nav_max_angular_speed:="$SIM_NAV_MAX_ANGULAR_SPEED" \
    trial_id:="validation_trial_${trial}" \
    run_id:="$run_id" \
    output_root:="$OUTPUT_ROOT" \
    >"/tmp/waver_bird_patrol_${trial}.launch.log" 2>&1 &
  launch_pid=$!

  sleep 12
  python3 "$WS/src/waver_patrol/scripts/waver_bird_patrol_trial_validator.py" \
    --required-removed 5 \
    --required-laps 2 \
    --timeout-sec "$TIMEOUT_SEC"
  result=$?
  if [ "$result" -ne 0 ]; then
    echo "[TRIAL $trial/$TRIALS] FAIL. Launch log: /tmp/waver_bird_patrol_${trial}.launch.log"
    kill "$launch_pid" 2>/dev/null || true
    cleanup_trial
    exit "$result"
  fi

  echo "[TRIAL $trial/$TRIALS] PASS"
  kill "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
done

cleanup_trial
echo "PASS: $TRIALS consecutive Waver bird patrol validation trials"
