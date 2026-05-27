#!/usr/bin/env bash
set -euo pipefail

if [ "${WAVER_WHEEL_ON:-false}" = "true" ]; then
  echo "ERROR: WAVER_WHEEL_ON=true. Refusing automatic cleanup on a wheel-on robot."
  exit 2
fi

echo "== Waver stale process cleanup =="

protected_pids=" $$ "
parent_pid="$PPID"
while [ -n "${parent_pid:-}" ] && [ "$parent_pid" != "0" ]; do
  protected_pids="${protected_pids}${parent_pid} "
  parent_pid="$(ps -o ppid= -p "$parent_pid" 2>/dev/null | tr -d ' ' || true)"
done

is_protected_pid() {
  case "$protected_pids" in
    *" $1 "*) return 0 ;;
    *) return 1 ;;
  esac
}

patterns=(
  "ros2 launch waver_patrol"
  "slam_gmapping"
  "slam_toolbox"
  "mapping_workflow_manager_node"
  "mission_patrol_manager_node"
  "safety_cmd_mux_node"
  "static_transform_publisher"
  "waver_remote_panel"
  "gazebo_map_path_visualizer_node"
  "moving_object_map_transform_node"
  "moving_object_motion_filter_node"
  "pointcloud_lidar_objects_node"
  "gzserver"
  "gzclient"
)

for pattern in "${patterns[@]}"; do
  pids="$(pgrep -f "$pattern" || true)"
  if [ -n "$pids" ]; then
    targets=""
    for pid in $pids; do
      if is_protected_pid "$pid"; then
        continue
      fi
      targets="${targets}${pid} "
    done
    if [ -n "$targets" ]; then
      echo "Stopping pattern [$pattern]: $targets"
      kill -TERM $targets 2>/dev/null || true
    fi
  fi
done

sleep 2

for pattern in "${patterns[@]}"; do
  pids="$(pgrep -f "$pattern" || true)"
  if [ -n "$pids" ]; then
    targets=""
    for pid in $pids; do
      if is_protected_pid "$pid"; then
        continue
      fi
      targets="${targets}${pid} "
    done
    if [ -n "$targets" ]; then
      echo "Force stopping pattern [$pattern]: $targets"
      kill -KILL $targets 2>/dev/null || true
    fi
  fi
done

echo "cleanup requested; run waver_clean_graph_check.sh before the next trial"
