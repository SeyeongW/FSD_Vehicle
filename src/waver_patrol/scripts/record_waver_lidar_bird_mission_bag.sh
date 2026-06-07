#!/usr/bin/env bash
set -euo pipefail

ROOT="${1:-$HOME/ros2_ws2/FSD_Vehicle/experiment_results/raw_bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${ROOT}/waver_lidar_bird_${STAMP}"
mkdir -p "${ROOT}"

ros2 bag record -o "${OUT}" \
  /tf /tf_static /odom /amcl_pose /scan /mid360_PointCloud2 \
  /waver/lidar_objects_map /waver/aerial_target /waver/aerial_target_active \
  /waver/moving_target_valid /waver/lidar_target_height_m /waver/lidar_target_range_m \
  /waver/lidar_target_velocity_mps /waver/camera_aim_target_pose /waver/camera_gimbal_cmd \
  /waver/camera_gimbal_feedback /waver/camera_target_centered /waver/body_tracking_state \
  /waver/body_target_centered /waver/bird_detections_2d /waver/bird_confirmed \
  /waver/target_class /waver/target_confidence /waver/sound_alert_request \
  /waver/sound_state /waver/target_departed /waver/mission_resume_request \
  /waver/mission_state /cmd_vel /waver/safety_state
