#!/usr/bin/env bash
set -euo pipefail

echo "== Waver clean graph check =="

topic_pub_count() {
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

fail=0
for topic in /cmd_vel /waver/mode /scan /scan_slam /scan_safety /map /odom; do
  count="$(topic_pub_count "$topic")"
  echo "$topic publishers=$count"
  if [ "$count" -gt 1 ]; then
    echo "ERROR: $topic has duplicate publishers"
    fail=1
  fi
done

cmd_publishers="$(publisher_nodes /cmd_vel)"
cmd_pubs="$(printf '%s\n' "$cmd_publishers" | sed '/^$/d' | awk 'END {print NR+0}')"
if [ "$cmd_pubs" -gt 0 ] && printf '%s\n' "$cmd_publishers" | grep -Ev '^safety_cmd_mux_node$' >/dev/null; then
  echo "ERROR: /cmd_vel publisher is not the Waver safety gate"
  printf '  publisher: %s\n' $cmd_publishers
  fail=1
fi

mode_publishers="$(publisher_nodes /waver/mode)"
mode_pubs="$(printf '%s\n' "$mode_publishers" | sed '/^$/d' | awk 'END {print NR+0}')"
if [ "$mode_pubs" -gt 0 ] && printf '%s\n' "$mode_publishers" | grep -Ev '^mission_patrol_manager_node$' >/dev/null; then
  echo "ERROR: /waver/mode publisher is not mission_patrol_manager_node"
  printf '  publisher: %s\n' $mode_publishers
  fail=1
fi

nodes="$(ros2 node list 2>/dev/null || true)"
for name in \
  /safety_cmd_mux_node \
  /mission_patrol_manager_node \
  /mapping_workflow_manager_node \
  /slam_gmapping \
  /async_slam_toolbox_node \
  /sync_slam_toolbox_node \
  /map_server \
  /controller_server \
  /planner_server \
  /bt_navigator
do
  count="$(printf '%s\n' "$nodes" | awk -v n="$name" '$0 == n {c++} END {print c+0}')"
  echo "$name nodes=$count"
  if [ "$count" -gt 1 ]; then
    echo "ERROR: duplicate node name $name"
    fail=1
  fi
done

if command -v timeout >/dev/null 2>&1; then
  tf_tmp="$(mktemp)"
  timeout 3 ros2 topic echo /tf_static --once >"$tf_tmp" 2>/dev/null || true
  if [ -s "$tf_tmp" ]; then
    pairs="$(awk '
      /frame_id:/ {parent=$2}
      /child_frame_id:/ {
        child=$2
        gsub("\"", "", parent)
        gsub("\"", "", child)
        if (parent != "" && child != "") print parent "->" child
      }' "$tf_tmp" | sort | uniq -d)"
    if [ -n "$pairs" ]; then
      echo "ERROR: duplicate /tf_static frame pairs:"
      printf '%s\n' "$pairs"
      fail=1
    fi
  fi
  rm -f "$tf_tmp"
fi

if [ "$fail" -ne 0 ]; then
  echo "FAIL: graph is not clean"
  exit 1
fi

echo "PASS: graph is clean"
