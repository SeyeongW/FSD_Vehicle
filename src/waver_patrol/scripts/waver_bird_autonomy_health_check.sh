#!/usr/bin/env bash
set -euo pipefail

echo "== Waver bird autonomy health =="
echo "-- detector"
timeout 3s ros2 topic echo --once /waver/bird_detector_state 2>/dev/null || echo "WARN: no detector state"
timeout 3s ros2 topic echo --once /waver/bird_confirmed 2>/dev/null || echo "WARN: no bird_confirmed"

echo "-- fusion"
timeout 3s ros2 topic echo --once /waver/bird_fusion_state 2>/dev/null || echo "WARN: no fusion state"
timeout 3s ros2 topic echo --once /waver/bird_target_valid 2>/dev/null || echo "WARN: no bird_target_valid"
timeout 3s ros2 topic echo --once /waver/target_goal_state 2>/dev/null || echo "WARN: no target_goal_state"

echo "-- 3D and motion sources"
timeout 4s ros2 topic hz /mid360_PointCloud2 2>/dev/null || echo "WARN: pointcloud hz unavailable"
timeout 4s ros2 topic hz /waver/lidar_objects 2>/dev/null || echo "WARN: lidar_objects hz unavailable"
timeout 4s ros2 topic hz /waver/elevated_dynamic_targets 2>/dev/null || echo "WARN: elevated targets hz unavailable"
timeout 4s ros2 topic hz /camera/image_raw 2>/dev/null || echo "WARN: camera hz unavailable"

echo "-- safety gates"
timeout 3s ros2 topic echo --once /waver/safety_state 2>/dev/null || echo "WARN: no safety state"
timeout 3s ros2 topic echo --once /waver/battery_safety_state 2>/dev/null || echo "WARN: no battery safety state"

echo "Done. A missing model should report MODEL_MISSING and bird_confirmed=false; that is safe but bird approach is disabled."
