#!/usr/bin/env bash
set -euo pipefail

OUT="${1:-$HOME/ros2_ws/waver_experiments/bags/waver_experiment_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"

ros2 bag record \
  /odom \
  /amcl_pose \
  /tf \
  /tf_static \
  /scan \
  /cmd_vel \
  /waver/cmd_vel_nav2 \
  /waver/mission_state \
  /waver/mission_event \
  /waver/radar_target_goal \
  /waver/radar_target_active \
  /waver/aerial_target \
  /waver/aerial_target_active \
  /waver/lidar_objects \
  /waver/lidar_objects_map \
  /waver/lidar_object_point_map \
  /waver/moving_object_map_transform_state \
  /waver/lidar_object_goal_preview \
  /waver/object_mission_goal \
  /waver/object_mission_goal_active \
  /waver/target_class \
  /waver/target_confidence \
  /waver/bird_confirmed \
  /waver/sound_alert_state \
  /waver/sound_task_done \
  /waver/safety_state \
  /battery_state \
  /voltage \
  -o "$OUT"
