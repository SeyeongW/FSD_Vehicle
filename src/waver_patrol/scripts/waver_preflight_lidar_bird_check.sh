#!/usr/bin/env bash
set -euo pipefail

fail=0
check_topic() {
  local topic="$1"
  if ! ros2 topic info "${topic}" >/dev/null 2>&1; then
    echo "WARN missing topic: ${topic}"
    fail=1
  else
    echo "OK topic: ${topic}"
  fi
}

publisher_count() {
  ros2 topic info -v "$1" 2>/dev/null | awk '/Publisher count:/ {print $3; exit}'
}

cmd_count="$(publisher_count /cmd_vel || echo 0)"
if [ "${cmd_count}" != "1" ]; then
  echo "FAIL /cmd_vel publisher count=${cmd_count}, expected 1"
  fail=1
else
  echo "OK /cmd_vel publisher count=1"
fi

check_topic /waver/mission_state
check_topic /waver/current_waypoint
check_topic /waver/emergency_stop
check_topic /waver/speed_limit
check_topic /waver/aerial_target_active
check_topic /waver/moving_target_valid
check_topic /waver/lidar_target_height_m
check_topic /waver/lidar_target_range_m
check_topic /waver/bird_detections_2d
check_topic /waver/bird_confirmed
check_topic /waver/camera_gimbal_cmd
check_topic /waver/camera_gimbal_feedback
check_topic /waver/mission_report

RESULT_ROOT="${WAVER_EXPERIMENT_RESULTS:-$HOME/ros2_ws5/FSD_Vehicle/experiment_results}"
mkdir -p "${RESULT_ROOT}"
if [ -w "${RESULT_ROOT}" ]; then
  echo "OK experiment_results writable: ${RESULT_ROOT}"
else
  echo "FAIL experiment_results not writable: ${RESULT_ROOT}"
  fail=1
fi

echo "INFO real gimbal output default must remain false"
echo "INFO sound output default must remain false"
echo "INFO external data send default must remain false"

exit "${fail}"
