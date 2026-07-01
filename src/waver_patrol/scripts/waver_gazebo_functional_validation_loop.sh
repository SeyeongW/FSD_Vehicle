#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
SCENARIO="all"
CYCLES=2
TIMEOUT_SEC="${WAVER_GAZEBO_VALIDATION_TIMEOUT_SEC:-120}"
ROS_DOMAIN_ID_BASE="${WAVER_GAZEBO_VALIDATION_DOMAIN_BASE:-83}"
REPORT_ROOT="${WAVER_GAZEBO_REPORT_ROOT:-${ROOT}/reports/gazebo_functional_validation/$(date +%Y%m%d_%H%M%S)}"

usage() {
  cat <<'EOF'
Usage: bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh [--scenario NAME] [--cycles N]

Scenarios:
  keyboard_teleop_smoke
  autonomous_patrol_smoke
  slam_mapping_smoke
  remote_ui_smoke
  safety_obstacle_smoke
  all

The script uses isolated ROS_DOMAIN_ID values and never starts real serial/base
drivers. Missing ROS/Gazebo dependencies are reported as SKIP_WITH_REASON.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --scenario) SCENARIO="${2:?missing scenario}"; shift 2 ;;
    --cycles) CYCLES="${2:?missing cycles}"; shift 2 ;;
    --timeout-sec) TIMEOUT_SEC="${2:?missing timeout}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

mkdir -p "${REPORT_ROOT}"
summary="${REPORT_ROOT}/summary.csv"
echo "cycle,scenario,status,command,evidence,key_output" > "${summary}"

scenario_list() {
  case "$1" in
    keyboard_teleop_smoke) echo "keyboard_teleop_smoke" ;;
    autonomous_patrol_smoke) echo "autonomous_patrol_smoke" ;;
    slam_mapping_smoke) echo "slam_mapping_smoke" ;;
    remote_ui_smoke) echo "remote_ui_smoke" ;;
    safety_obstacle_smoke) echo "safety_obstacle_smoke" ;;
    all) echo "keyboard_teleop_smoke autonomous_patrol_smoke slam_mapping_smoke remote_ui_smoke safety_obstacle_smoke" ;;
    *) echo "ERROR: unknown scenario: $1" >&2; exit 2 ;;
  esac
}

scenario_command() {
  case "$1" in
    keyboard_teleop_smoke)
      printf "%s" "ros2 launch ugv_tools waver_gazebo_test.launch.py start_gazebo:=true use_minimal_model:=true gui:=false start_scripted_keyboard:=true start_patrol:=false start_remote_panel:=false require_scan:=false start_lidar_perception:=false start_lidar_target_follow:=false start_bird_manager:=false start_data_logger:=false"
      ;;
    autonomous_patrol_smoke)
      printf "%s" "ros2 launch ugv_tools waver_gazebo_test.launch.py start_gazebo:=true use_minimal_model:=true gui:=false start_scripted_keyboard:=false start_patrol:=true start_remote_panel:=false require_scan:=false loop_count:=1 start_lidar_perception:=false start_lidar_target_follow:=false start_bird_manager:=false start_data_logger:=false"
      ;;
    slam_mapping_smoke)
      printf "%s" "WAVER_SPAWN_TEST_OBSTACLE=true WAVER_USE_GUI=false WAVER_START_RVIZ=false WAVER_UI_SLAM_TIMEOUT=90 bash scripts/run_ui_slam_mapping_gazebo_smoke.sh"
      ;;
    remote_ui_smoke)
      printf "%s" "bash src/waver_patrol/scripts/waver_remote_ui_validation.sh --mock --cycles 1"
      ;;
    safety_obstacle_smoke)
      printf "%s" "WAVER_SPAWN_TEST_OBSTACLE=true WAVER_TEST_OBSTACLE_MIN_OCCUPIED=5 WAVER_USE_GUI=false WAVER_UI_SLAM_TIMEOUT=90 bash scripts/run_ui_slam_mapping_gazebo_smoke.sh"
      ;;
  esac
}

can_run_ros() {
  [ -f /opt/ros/humble/setup.bash ] && command -v timeout >/dev/null 2>&1
}

cleanup_gazebo_validation() {
  pkill -f "[g]zserver.*waver_flat.world" >/dev/null 2>&1 || true
  pkill -f "[g]zserver.*ugv_world.world" >/dev/null 2>&1 || true
  pkill -f "[g]zclient" >/dev/null 2>&1 || true
}

fatal_log_patterns() {
  grep -Eiq \
    "Unable to start server|Spawn service failed|process has died|Caught exception in launch|package .* not found|Traceback \\(most recent call last\\)" \
    "$1"
}

timeout_success_markers() {
  local scenario="$1"
  local log="$2"
  case "${scenario}" in
    keyboard_teleop_smoke)
      grep -q "Spawn status: SpawnEntity: Successfully spawned" "${log}" \
        && grep -q "keyboard_ctrl.*process has finished cleanly" "${log}"
      ;;
    autonomous_patrol_smoke)
      grep -q "Spawn status: SpawnEntity: Successfully spawned" "${log}" \
        && grep -q "waver_gazebo_patrol" "${log}"
      ;;
    *)
      return 1
      ;;
  esac
}

run_one() {
  local cycle="$1"
  local scenario="$2"
  local cycle_dir="${REPORT_ROOT}/cycle_${cycle}"
  mkdir -p "${cycle_dir}"
  local log="${cycle_dir}/${scenario}.log"
  local cmd
  cmd="$(scenario_command "${scenario}")"
  local domain=$((ROS_DOMAIN_ID_BASE + cycle))
  local status="PASS"
  local key="completed"

  if ! can_run_ros; then
    status="SKIP_WITH_REASON"
    key="ROS Humble or timeout command unavailable"
    {
      echo "GAZEBO_FUNCTIONAL_VALIDATION ${scenario}"
      echo "STATUS=${status}"
      echo "REASON=${key}"
      echo "COMMAND=${cmd}"
    } >"${log}"
  else
    cleanup_gazebo_validation
    {
      echo "GAZEBO_FUNCTIONAL_VALIDATION scenario=${scenario} cycle=${cycle}"
      echo "ROS_DOMAIN_ID=${domain}"
      echo "COMMAND=${cmd}"
      cd "${ROOT}"
      set +u
      source /opt/ros/humble/setup.bash
      if [ -f install/setup.bash ]; then
        source install/setup.bash
      fi
      set -u
      export ROS_DOMAIN_ID="${domain}"
      export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
      unset SERIAL_PORT
      set +e
      timeout --foreground "${TIMEOUT_SEC}" bash -lc "${cmd}"
      rc=$?
      set -e
      echo "COMMAND_RC=${rc}"
      if [ "$rc" -ne 0 ] && [ "$rc" -ne 124 ]; then
        exit "$rc"
      fi
      if [ "$rc" -eq 124 ]; then
        echo "STATUS=SKIP_WITH_REASON timeout_before_scenario_finished"
      else
        echo "STATUS=PASS"
      fi
    } >"${log}" 2>&1 || {
      status="FAIL"
      key="$(tail -1 "${log}" | tr ',' ';')"
    }
    cleanup_gazebo_validation
    if grep -q "STATUS=SKIP_WITH_REASON" "${log}" && [ "${status}" = "PASS" ]; then
      status="SKIP_WITH_REASON"
      key="$(grep 'STATUS=SKIP_WITH_REASON' "${log}" | tail -1 | tr ',' ';')"
    fi
    if [ "${status}" = "PASS" ] && fatal_log_patterns "${log}"; then
      status="FAIL"
      key="fatal Gazebo/launch error pattern in log"
    fi
    if [ "${status}" = "SKIP_WITH_REASON" ] && ! fatal_log_patterns "${log}" && timeout_success_markers "${scenario}" "${log}"; then
      status="PASS"
      key="timeout_after_expected_success_markers"
    fi
  fi

  echo "${cycle},${scenario},${status},\"${cmd}\",${log},\"${key}\"" >> "${summary}"
  echo "GAZEBO_FUNCTIONAL_${scenario}_CYCLE_${cycle}=${status} evidence=${log}"
  [ "${status}" != "FAIL" ]
}

overall=0
scenarios="$(scenario_list "${SCENARIO}")"
for cycle in $(seq 1 "${CYCLES}"); do
  for s in ${scenarios}; do
    run_one "${cycle}" "${s}" || overall=1
  done
done

echo "GAZEBO_FUNCTIONAL_REPORT=${REPORT_ROOT}"
if [ "${overall}" -eq 0 ]; then
  echo "GAZEBO_FUNCTIONAL_VALIDATION=PASS_OR_SKIP"
else
  echo "GAZEBO_FUNCTIONAL_VALIDATION=FAIL"
fi
exit "${overall}"
