#!/usr/bin/env bash
set -eo pipefail

# Waver real-robot preflight check.
# This script does not publish motion commands. It only checks workspace,
# ROS graph, sensor topics, and serial ownership before a real-robot run.

WS="${WAVER_WS:-$HOME/ros2_ws/FSD_Vehicle}"
cd "$WS"

echo "== Waver real preflight check =="
echo "workspace: $WS"
date

BRANCH="$(git branch --show-current 2>/dev/null || true)"
if [ "$BRANCH" != "jo" ]; then
  echo "ERROR: expected git branch jo, got '${BRANCH:-unknown}'"
  exit 10
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "ERROR: /opt/ros/humble/setup.bash not found"
  exit 1
fi

if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
else
  echo "WARN: $WS/install/setup.bash not found. Build the workspace first."
fi

echo
echo "== Package path check =="
WAVER_PATHS="$(colcon list --paths-only 2>/dev/null | grep '/waver_patrol$' || true)"
echo "$WAVER_PATHS"
WAVER_COUNT="$(printf '%s\n' "$WAVER_PATHS" | sed '/^$/d' | wc -l)"
if [ "$WAVER_COUNT" -ne 1 ]; then
  echo "ERROR: expected exactly one build-visible waver_patrol package, got $WAVER_COUNT"
  echo "Fix duplicate waver_patrol paths before running on the real robot."
  exit 2
fi
ros2 pkg prefix waver_patrol || true

echo
echo "== Real-profile forbidden node check =="
FORBIDDEN="$(ros2 node list 2>/dev/null | grep -E 'deep_learning_bridge_stub|fake_camera|gazebo_bird|bird_detection_pipeline|simple_sim_odom|test_publisher' || true)"
if [ -n "$FORBIDDEN" ]; then
  echo "$FORBIDDEN"
  echo "ERROR: fake/test/Gazebo-only nodes are active in a real preflight."
  exit 11
fi

echo
echo "== Active ROS nodes =="
ros2 node list 2>/dev/null | sort || true

echo
echo "== /cmd_vel ownership =="
CMD_INFO="$(ros2 topic info -v /cmd_vel 2>/dev/null || true)"
if [ -z "$CMD_INFO" ]; then
  echo "WARN: /cmd_vel is not visible yet. Start Waver launch, then rerun this script."
else
  echo "$CMD_INFO"
  PUBLISHER_COUNT="$(printf '%s\n' "$CMD_INFO" | grep -c 'Endpoint type: PUBLISHER' || true)"
  if [ "$PUBLISHER_COUNT" -gt 1 ]; then
    echo "ERROR: /cmd_vel has more than one publisher. Do not enable serial."
    exit 3
  fi
  if [ "$PUBLISHER_COUNT" -eq 1 ] && ! printf '%s\n' "$CMD_INFO" | grep -q 'safety_cmd_mux_node'; then
    echo "ERROR: /cmd_vel publisher is not safety_cmd_mux_node. Do not enable serial."
    exit 4
  fi
fi

echo
echo "== Safety topics =="
ros2 topic list 2>/dev/null | grep -E '^/waver/(safety_state|emergency_stop|external_stop|mode|serial_bridge_state)$|^/cmd_vel$' || true
timeout 3s ros2 topic echo --once /waver/safety_state 2>/dev/null || echo "WARN: no /waver/safety_state sample"
timeout 3s ros2 topic echo --once /waver/battery_safety_state 2>/dev/null || echo "WARN: no /waver/battery_safety_state sample"
timeout 3s ros2 topic echo --once /waver/bird_detector_state 2>/dev/null || echo "WARN: no /waver/bird_detector_state sample"
timeout 3s ros2 topic echo --once /waver/bird_fusion_state 2>/dev/null || echo "WARN: no /waver/bird_fusion_state sample"

echo
echo "== Sensor topics =="
ros2 topic list 2>/dev/null | grep -E 'scan|cloud|PointCloud|detections|lidar_objects|battery_state|voltage|odom|amcl_pose|tf' || true
timeout 4s ros2 topic hz /scan 2>/dev/null || echo "WARN: /scan hz unavailable"
timeout 4s ros2 topic hz /odom 2>/dev/null || echo "WARN: /odom hz unavailable"
timeout 4s ros2 topic hz /mid360_PointCloud2 2>/dev/null || echo "WARN: /mid360_PointCloud2 hz unavailable"
timeout 4s ros2 topic hz /camera/image_raw 2>/dev/null || echo "WARN: /camera/image_raw hz unavailable"

echo
echo "== Serial/process conflict check =="
ps aux | grep -E 'app.py|ugv_driver|serial_cmd_vel_bridge|ttyTHS0|ttyUSB|ttyACM' | grep -v grep || true
for dev in /dev/ttyTHS0 /dev/serial0 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
  if [ -e "$dev" ]; then
    echo "-- $dev"
    lsof "$dev" 2>/dev/null || echo "not currently owned"
  fi
done

echo
echo "== Reminder =="
echo "Real driving requires: require_scan=true, test publishers off, sound output off,"
echo "and exactly one final /cmd_vel publisher: safety_cmd_mux_node."
echo "Only enable start_serial_bridge=true after wheel-off direction and stop tests."
