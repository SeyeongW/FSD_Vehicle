#!/usr/bin/env bash
set -euo pipefail

echo "== Waver mapping health =="
for topic in /clock /scan /odom /map /waver/mapping_state /waver/map_apply_state /waver/mapping_active /waver/mapping_path; do
  echo "-- $topic"
  ros2 topic info -v "$topic" 2>/dev/null || echo "not visible"
done

echo "-- nodes"
ros2 node list 2>/dev/null | grep -E 'slam|mapping|map_server|waver_remote|safety|mission' || true

echo "-- tf odom -> base_link"
timeout 3 ros2 run tf2_ros tf2_echo odom base_link >/tmp/waver_tf_odom_base.txt 2>&1 && {
  head -20 /tmp/waver_tf_odom_base.txt
  echo "TF_ODOM_BASE=OK"
} || {
  cat /tmp/waver_tf_odom_base.txt
  echo "TF_ODOM_BASE=FAIL"
}

echo "-- /map hz sample"
timeout 8 ros2 topic hz /map 2>/dev/null || true
