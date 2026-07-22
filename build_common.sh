#!/bin/bash
# 공통 빌드 스크립트 (코드 수정 후 매번 사용)

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[build_common] 워크스페이스 루트: $WS_ROOT"
cd "$WS_ROOT"

# ROS2 환경 소싱 (필수)
source /opt/ros/humble/setup.bash

colcon build --packages-select \
    costmap_converter_msgs costmap_converter \
    emcl2 \
    ldlidar \
    robot_pose_publisher \
    teb_msgs teb_local_planner \
    ros2_livox_simulation livox_ros_driver2 \
    ugv_base_node ugv_interface \
    --cmake-args -DHUMBLE_ROS=humble

colcon build --packages-select \
    ugv_description ugv_gazebo \
    --symlink-install

source install/setup.bash
echo "[build_common] 완료!"
