#!/usr/bin/env bash
set -euo pipefail

STRICT=false
for arg in "$@"; do
  case "$arg" in
    --strict) STRICT=true ;;
    --help|-h)
      echo "Usage: $0 [--strict]"
      exit 0
      ;;
    *) echo "WARN: unknown argument ignored: $arg" ;;
  esac
done

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

publisher_nodes() {
  ros2 topic info -v "$1" 2>/dev/null | awk '
    /^Node name:/ {node=$3}
    /^Endpoint type:/ && $3 == "PUBLISHER" && node != "" {
      print node
      node=""
    }
  ' || true
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -qx "$1"
}

sample_topic() {
  local topic="$1"
  echo
  echo "== $topic =="
  if ! topic_exists "$topic"; then
    warn_or_fail "$topic is not visible"
    return
  fi
  timeout 3s ros2 topic echo --once "$topic" 2>/dev/null || warn_or_fail "$topic has no sample"
}

count_publishers() {
  publisher_nodes "$1" | sed '/^$/d' | awk 'END {print NR+0}'
}

echo "== Waver indoor patrol status =="
echo "strict: $STRICT"
date

echo
echo "== /cmd_vel authority =="
ros2 topic info -v /cmd_vel 2>/dev/null || warn_or_fail "/cmd_vel is not visible"
CMD_NODES="$(publisher_nodes /cmd_vel)"
CMD_COUNT="$(printf '%s\n' "$CMD_NODES" | sed '/^$/d' | awk 'END {print NR+0}')"
echo "publishers=$CMD_COUNT"
printf '%s\n' "$CMD_NODES" | sed '/^$/d' | sed 's/^/publisher: /'
if [ "$CMD_COUNT" -ne 1 ]; then
  warn_or_fail "/cmd_vel publisher count must be exactly 1"
elif ! printf '%s\n' "$CMD_NODES" | grep -q '^safety_cmd_mux_node$'; then
  warn_or_fail "/cmd_vel publisher must be safety_cmd_mux_node"
fi

echo
echo "== Topic visibility =="
for topic in /scan /scan_safety /odom /waver/mode /waver/safety_state /waver/mission_state /waver/livox_scan_adapter_state; do
  if topic_exists "$topic"; then
    echo "OK $topic"
  else
    case "$topic" in
      /scan_safety) echo "INFO $topic not visible; /scan may be used instead" ;;
      *) warn_or_fail "$topic not visible" ;;
    esac
  fi
done

echo
echo "== Scan rate =="
SCAN_TOPIC="/scan"
if topic_exists /scan_safety; then
  SCAN_TOPIC="/scan_safety"
fi
timeout 5s ros2 topic hz "$SCAN_TOPIC" 2>/dev/null || warn_or_fail "$SCAN_TOPIC hz unavailable"

sample_topic /waver/safety_state
sample_topic /waver/mode
sample_topic /waver/mission_state
sample_topic /waver/livox_scan_adapter_state

echo
echo "== /odom visibility =="
ros2 topic info -v /odom 2>/dev/null || warn_or_fail "/odom info unavailable"

echo
if [ "$FAILURES" -ne 0 ]; then
  echo "INDOOR_PATROL_STATUS=FAIL failures=$FAILURES"
  exit 1
fi
echo "INDOOR_PATROL_STATUS=PASS"
