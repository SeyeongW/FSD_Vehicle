#!/usr/bin/env bash
set -euo pipefail

STRICT=false
WHEEL_ON=false
for arg in "$@"; do
  case "$arg" in
    --strict) STRICT=true ;;
    --wheel-on) WHEEL_ON=true ;;
    *) echo "WARN: unknown argument ignored: $arg" ;;
  esac
done

REPO="${WAVER_REPO:-$HOME/ros2_ws5/FSD_Vehicle}"
WS="${WAVER_WS:-$REPO}"
cd "$REPO"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi
if [ -f "$WS/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  set -u
elif [ -f "$REPO/install/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "$REPO/install/setup.bash"
  set -u
fi

FAILURES=0
fail() {
  echo "FAIL: $*"
  FAILURES=$((FAILURES + 1))
}
warn_or_fail() {
  if $STRICT; then
    fail "$*"
  else
    echo "WARN: $*"
  fi
}

count_publishers() {
  local topic="$1"
  publisher_nodes "$topic" | awk 'END {print NR+0}'
}

topic_info() {
  ros2 topic info -v "$1" 2>/dev/null || true
}

publisher_nodes() {
  ros2 topic info -v "$1" 2>/dev/null | awk '
    /^Node name:/ {node=$3}
    /^Endpoint type:/ && $3 == "PUBLISHER" && node != "" {
      print node
      node=""
    }
  ' || true
}

POINTCLOUD_TOPIC="${WAVER_POINTCLOUD_TOPIC:-${POINTCLOUD_TOPIC:-/livox/lidar}}"
CAMERA_IMAGE_TOPIC="${WAVER_CAMERA_IMAGE_TOPIC:-${CAMERA_IMAGE_TOPIC:-/camera/image_raw}}"
LIVOX_FRAME="${WAVER_LIVOX_FRAME:-${LIVOX_FRAME_ID:-livox}}"
CAMERA_FRAME="${WAVER_CAMERA_FRAME:-${CAMERA_FRAME:-camera_link}}"
PREFLIGHT_PROFILE="${WAVER_PREFLIGHT_PROFILE:-indoor_patrol}"
REQUIRE_POINTCLOUD="${WAVER_PREFLIGHT_REQUIRE_POINTCLOUD:-1}"
REQUIRE_CAMERA="${WAVER_PREFLIGHT_REQUIRE_CAMERA:-0}"
REQUIRE_BIRD="${WAVER_PREFLIGHT_REQUIRE_BIRD:-0}"
REQUIRE_BATTERY="${WAVER_PREFLIGHT_REQUIRE_BATTERY:-0}"

echo "== Waver real preflight check =="
echo "workspace: $WS"
echo "repo: $REPO"
echo "strict: $STRICT wheel_on: $WHEEL_ON"
echo "profile: $PREFLIGHT_PROFILE"
echo "requires: scan=1 odom=1 pointcloud=$REQUIRE_POINTCLOUD camera=$REQUIRE_CAMERA bird=$REQUIRE_BIRD battery=$REQUIRE_BATTERY"
date

BRANCH="$(git branch --show-current 2>/dev/null || true)"
[ "$BRANCH" = "jo" ] || fail "expected git branch jo, got '${BRANCH:-unknown}'"

if [ -x "$REPO/src/waver_patrol/scripts/waver_duplicate_package_check.sh" ]; then
  "$REPO/src/waver_patrol/scripts/waver_duplicate_package_check.sh" "$WS" || fail "duplicate package check failed"
else
  warn_or_fail "duplicate package check script missing"
fi

echo
echo "== Forbidden real-profile nodes =="
FORBIDDEN="$(ros2 node list 2>/dev/null | grep -E 'deep_learning_bridge_stub|fake_camera|gazebo_bird|bird_detection_pipeline|simple_sim_odom|test_publisher|gazebo_live_mapping' || true)"
if [ -n "$FORBIDDEN" ]; then
  echo "$FORBIDDEN"
  fail "fake/test/Gazebo-only node active"
fi

echo
echo "== /cmd_vel ownership =="
CMD_INFO="$(topic_info /cmd_vel)"
echo "${CMD_INFO:-/cmd_vel not visible}"
CMD_PUB_NODES="$(publisher_nodes /cmd_vel)"
CMD_PUBS="$(printf '%s\n' "$CMD_PUB_NODES" | sed '/^$/d' | awk 'END {print NR+0}')"
if [ "$CMD_PUBS" -ne 1 ]; then
  warn_or_fail "/cmd_vel publisher count must be 1, got $CMD_PUBS"
elif ! printf '%s\n' "$CMD_PUB_NODES" | grep -q '^safety_cmd_mux_node$'; then
  fail "final /cmd_vel publisher is not safety_cmd_mux_node"
fi
if printf '%s\n' "$CMD_PUB_NODES" | grep -E 'controller_server|waver_remote_panel|teleop|ugv_driver|target_goal|pointcloud' >/dev/null; then
  fail "direct /cmd_vel publisher violation"
fi

MODE_PUBS="$(count_publishers /waver/mode)"
if [ "$MODE_PUBS" -ne 1 ]; then
  warn_or_fail "/waver/mode publisher count must be 1, got $MODE_PUBS"
else
  MODE_PUB_NODES="$(publisher_nodes /waver/mode)"
  if ! printf '%s\n' "$MODE_PUB_NODES" | grep -q '^mission_patrol_manager_node$'; then
    fail "/waver/mode publisher is not mission_patrol_manager_node"
  fi
fi

SCAN_PUBS="$(count_publishers /scan)"
SCAN_SAFETY_PUBS="$(count_publishers /scan_safety)"
if [ "$SCAN_PUBS" -ne 1 ] && [ "$SCAN_SAFETY_PUBS" -ne 1 ]; then
  warn_or_fail "/scan or /scan_safety publisher count must be 1, got /scan=$SCAN_PUBS /scan_safety=$SCAN_SAFETY_PUBS"
fi
ODOM_PUBS="$(count_publishers /odom)"
if [ "$ODOM_PUBS" -ne 1 ]; then
  warn_or_fail "/odom publisher count must be 1, got $ODOM_PUBS"
fi

echo
echo "== Serial owner check =="
SERIAL_NODE_COUNT="$(ros2 node list 2>/dev/null | grep -E 'waver_base_driver_node|serial_cmd_vel_bridge|waver_cmd_vel_serial_bridge|ugv_bringup|ugv_driver' | wc -l)"
echo "serial-related node count: $SERIAL_NODE_COUNT"
if [ "$SERIAL_NODE_COUNT" -gt 1 ]; then
  fail "serial owner count must be <=1"
fi
for dev in /dev/serial/by-id/* /dev/ttyTHS0 /dev/ttyTHS1 /dev/serial0 /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0 /dev/ttyACM1; do
  [ -e "$dev" ] || continue
  OWNERS="$(lsof "$dev" 2>/dev/null | tail -n +2 | wc -l || true)"
  echo "$dev owners=$OWNERS"
  if [ "$OWNERS" -gt 1 ]; then
    fail "serial device has multiple owners: $dev"
  fi
done

echo
echo "== Sensor Hz checks =="
check_hz() {
  local topic="$1"
  local min_hz="$2"
  local out
  out="$(timeout 5s ros2 topic hz "$topic" 2>/dev/null || true)"
  echo "-- $topic"
  echo "${out:-no samples}"
  local hz
  hz="$(printf '%s\n' "$out" | awk '/average rate:/ {print $3; exit}')"
  if [ -z "$hz" ]; then
    warn_or_fail "$topic hz unavailable"
    return
  fi
  python3 - "$hz" "$min_hz" "$topic" <<'PY' || fail "$topic hz below threshold"
import sys
hz=float(sys.argv[1]); min_hz=float(sys.argv[2]); topic=sys.argv[3]
if hz < min_hz:
    print(f"{topic}: {hz:.3f} < {min_hz:.3f}")
    raise SystemExit(1)
PY
}

if [ "$SCAN_SAFETY_PUBS" -eq 1 ]; then
  check_hz /scan_safety 5.0
else
  check_hz /scan 5.0
fi
check_hz /odom 5.0
if [ "$REQUIRE_POINTCLOUD" = "1" ]; then
  check_hz "$POINTCLOUD_TOPIC" 3.0
else
  echo "-- $POINTCLOUD_TOPIC"
  echo "SKIP: pointcloud is optional for profile=$PREFLIGHT_PROFILE"
fi
if [ "$REQUIRE_CAMERA" = "1" ]; then
  check_hz "$CAMERA_IMAGE_TOPIC" 3.0
else
  echo "-- $CAMERA_IMAGE_TOPIC"
  echo "SKIP: camera is optional for profile=$PREFLIGHT_PROFILE"
fi

echo
echo "== TF checks =="
check_tf() {
  local from="$1"
  local to="$2"
  echo "-- $from -> $to"
  if ! timeout 4s ros2 run tf2_ros tf2_echo "$from" "$to" >/tmp/waver_tf_check.txt 2>&1; then
    cat /tmp/waver_tf_check.txt || true
    warn_or_fail "TF failed: $from -> $to"
  else
    head -n 5 /tmp/waver_tf_check.txt || true
  fi
}
check_tf map odom
check_tf odom base_link
if [ "$REQUIRE_POINTCLOUD" = "1" ]; then
  check_tf base_link "$LIVOX_FRAME"
else
  echo "-- base_link -> $LIVOX_FRAME"
  echo "SKIP: pointcloud TF is optional for profile=$PREFLIGHT_PROFILE"
fi
if [ "$REQUIRE_CAMERA" = "1" ]; then
  check_tf base_link "$CAMERA_FRAME"
else
  echo "-- base_link -> $CAMERA_FRAME"
  echo "SKIP: camera TF is optional for profile=$PREFLIGHT_PROFILE"
fi

echo
echo "== State topic checks =="
sample_topic() {
  local topic="$1"
  local data
  data="$(timeout 3s ros2 topic echo --once "$topic" 2>/dev/null || true)"
  echo "-- $topic"
  echo "${data:-no sample}"
  if [ -z "$data" ]; then
    warn_or_fail "$topic missing"
  fi
  printf '%s\n' "$data"
}
SAFETY="$(sample_topic /waver/safety_state || true)"
if [ "$REQUIRE_BATTERY" = "1" ]; then
  BATTERY="$(sample_topic /waver/battery_safety_state || true)"
else
  echo "-- /waver/battery_safety_state"
  echo "SKIP: battery safety state is optional for profile=$PREFLIGHT_PROFILE"
  BATTERY=""
fi
if [ "$REQUIRE_BIRD" = "1" ]; then
  sample_topic /waver/bird_detector_state >/dev/null || true
  sample_topic /waver/bird_fusion_state >/dev/null || true
else
  echo "-- /waver/bird_detector_state"
  echo "SKIP: bird detector/fusion is optional for profile=$PREFLIGHT_PROFILE"
fi

if printf '%s\n' "$SAFETY" | grep -E 'EMERGENCY|FAULT|SCAN_.*STOP|UNKNOWN_MODE_STOP|MODE_(EMERGENCY|DISABLED)_STOP' >/dev/null; then
  warn_or_fail "safety_state indicates unsafe stop/fault"
fi
if [ "$REQUIRE_BATTERY" = "1" ] && printf '%s\n' "$SAFETY" | grep -E 'BATTERY_STALE_STOP' >/dev/null; then
  warn_or_fail "safety_state indicates battery stale stop"
fi
if [ "$REQUIRE_BATTERY" = "1" ] && printf '%s\n' "$BATTERY" | grep -E 'STALE|CRITICAL' >/dev/null; then
  warn_or_fail "battery safety state stale/critical"
fi

if $WHEEL_ON && ! $STRICT; then
  fail "wheel-on preflight requires --strict"
fi

echo
if [ "$FAILURES" -ne 0 ]; then
  echo "PRECHECK FAIL: $FAILURES failure(s)"
  exit 1
fi
echo "PRECHECK PASS"
