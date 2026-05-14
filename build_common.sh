#!/bin/bash
# Jetson 하드웨어 전용 빌드 스크립트 (코드 수정 후 매번 사용)
# 시뮬레이션(Gazebo, livox_laser_simulation) 및 unitree 관련 패키지 제외

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[build_common] 워크스페이스 루트: $WS_ROOT"
cd "$WS_ROOT"

# ROS2 환경 소싱 (필수)
source /opt/ros/humble/setup.bash

# RMW 설정 (dustynv Jetson 이미지는 cyclonedds만 포함)
# export 방식은 colcon 하위 프로세스에 불안정 → 인라인 주입 사용
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 1. 외부 패키지 빌드 (Jetson 전용 — 시뮬레이션 제외)
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon build --packages-select \
    apriltag apriltag_msgs apriltag_ros \
    cartographer \
    costmap_converter_msgs costmap_converter \
    emcl2 \
    openslam_gmapping slam_gmapping \
    ldlidar \
    rf2o_laser_odometry \
    robot_pose_publisher \
    teb_msgs teb_local_planner \
    vizanti vizanti_cpp vizanti_demos vizanti_msgs vizanti_server \
    livox_ros_driver2 \
    ugv_base_node ugv_interface \
    --cmake-args -DHUMBLE_ROS=humble -DBUILD_TESTING=OFF

# 2. C++ 메인 패키지 빌드
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon build --packages-select \
    ugv_description ugv_nav ugv_slam \
    --cmake-args -DHUMBLE_ROS=humble -DBUILD_TESTING=OFF

# 3. Python 메인 패키지 빌드 (--symlink-install 제외 — 구버전 setuptools 호환)
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon build --packages-select \
    ugv_bringup ugv_chat_ai \
    ugv_tools ugv_vision ugv_web_app ugv_lidar_detection

# 4. Jetson 전용 포인트클라우드 패키지 빌드
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp colcon build --packages-select \
    pcd_cluster_pkg pcd_to_scan_pkg plane_fit_pkg

source install/setup.bash
echo "[build_common] 완료!"
