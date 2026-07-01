#!/usr/bin/env bash
set -euo pipefail

echo "== Waver command-chain audit =="
ALLOW_COLLISION_MONITOR_FINAL="${WAVER_ALLOW_COLLISION_MONITOR_FINAL:-0}"
publisher_nodes_from_file() {
  awk '
    /^Node name:/ {node=$3}
    /^Endpoint type:/ && $3 == "PUBLISHER" && node != "" {
      print node
      node=""
    }
  ' "$1"
}

ros2 topic info -v /cmd_vel > /tmp/waver_cmd_vel_info.txt 2>/dev/null || {
  echo "ERROR: /cmd_vel is not visible"
  exit 1
}
cat /tmp/waver_cmd_vel_info.txt
PUB_NODES="$(publisher_nodes_from_file /tmp/waver_cmd_vel_info.txt || true)"
PUB_COUNT="$(printf '%s\n' "$PUB_NODES" | sed '/^$/d' | awk 'END {print NR+0}')"
if [ "$PUB_COUNT" -ne 1 ]; then
  echo "ERROR: expected exactly one /cmd_vel publisher, got $PUB_COUNT"
  exit 1
fi
if printf '%s\n' "$PUB_NODES" | grep -E 'controller_server|waver_remote_panel|teleop|ugv_driver|pointcloud|target_goal'; then
  echo "ERROR: direct /cmd_vel publisher violation detected"
  exit 2
fi
if [ "$ALLOW_COLLISION_MONITOR_FINAL" = "1" ]; then
  if ! printf '%s\n' "$PUB_NODES" | grep -E '^(safety_cmd_mux_node|collision_monitor)$' >/dev/null; then
    echo "ERROR: final /cmd_vel publisher is not an allowed safety gate"
    exit 3
  fi
elif ! printf '%s\n' "$PUB_NODES" | grep -q '^safety_cmd_mux_node$'; then
  echo "ERROR: final /cmd_vel publisher is not safety_cmd_mux_node"
  exit 3
fi

echo
echo "== /waver/mode authority =="
ros2 topic info -v /waver/mode > /tmp/waver_mode_info.txt 2>/dev/null || {
  echo "ERROR: /waver/mode is not visible"
  exit 4
}
cat /tmp/waver_mode_info.txt
MODE_PUB_NODES="$(publisher_nodes_from_file /tmp/waver_mode_info.txt || true)"
MODE_PUB_COUNT="$(printf '%s\n' "$MODE_PUB_NODES" | sed '/^$/d' | awk 'END {print NR+0}')"
if [ "$MODE_PUB_COUNT" -ne 1 ]; then
  echo "ERROR: expected exactly one /waver/mode publisher, got $MODE_PUB_COUNT"
  exit 4
fi
if ! printf '%s\n' "$MODE_PUB_NODES" | grep -q '^mission_patrol_manager_node$'; then
  echo "ERROR: /waver/mode publisher is not mission_patrol_manager_node"
  exit 4
fi
if printf '%s\n' "$MODE_PUB_NODES" | grep -q '^mapping_workflow_manager_node$'; then
  echo "ERROR: mapping_workflow_manager_node must not publish /waver/mode"
  exit 4
fi

echo
echo "== /scan and /odom publisher uniqueness =="
for topic in /scan /odom; do
  name="$(printf '%s' "$topic" | tr '/' '_')"
  file="/tmp/waver_${name}_info.txt"
  ros2 topic info -v "$topic" > "$file" 2>/dev/null || {
    echo "ERROR: $topic is not visible"
    exit 5
  }
  cat "$file"
  COUNT="$(grep -c 'Endpoint type: PUBLISHER' "$file" || true)"
  if [ "$COUNT" -ne 1 ]; then
    echo "ERROR: expected exactly one $topic publisher, got $COUNT"
    exit 5
  fi
done

echo
echo "== Candidate command topics =="
ros2 topic info -v /waver/manual_cmd_vel 2>/dev/null || true
ros2 topic info -v /waver/cmd_vel_nav2_raw 2>/dev/null || true
ros2 topic info -v /waver/cmd_vel_nav2 2>/dev/null || true
ros2 topic info -v /waver/cmd_vel_nav2_smooth 2>/dev/null || true

echo
echo "PASS: final /cmd_vel has one safety publisher"
