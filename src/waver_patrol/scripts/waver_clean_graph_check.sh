#!/usr/bin/env bash
set -euo pipefail

echo "== Waver clean graph check =="

topic_pub_count() {
  local topic="$1"
  local tmp
  tmp="$(mktemp)"
  if ! ros2 topic info -v "$topic" >"$tmp" 2>/dev/null; then
    rm -f "$tmp"
    echo 0
    return
  fi
  grep -c 'Endpoint type: PUBLISHER' "$tmp" || true
  rm -f "$tmp"
}

fail=0
for topic in /cmd_vel /waver/mode /scan /map /odom; do
  count="$(topic_pub_count "$topic")"
  echo "$topic publishers=$count"
  if [ "$count" -gt 1 ]; then
    echo "ERROR: $topic has duplicate publishers"
    fail=1
  fi
done

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
