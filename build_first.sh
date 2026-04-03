#!/bin/bash
# 전체 빌드 스크립트 (최초 1회 실행)

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[build_first] 워크스페이스 루트: $WS_ROOT"
cd "$WS_ROOT"

# ROS2 환경 소싱 (필수)
source /opt/ros/humble/setup.bash

# 기존 빌드 캐시가 다른 경로에서 만들어졌으면 자동 삭제
if [ -f "build/apriltag/CMakeCache.txt" ]; then
    CACHED_PATH=$(grep "CMAKE_CACHEFILE_DIR" build/apriltag/CMakeCache.txt 2>/dev/null | cut -d= -f2)
    if [ "$CACHED_PATH" != "$WS_ROOT/build/apriltag" ]; then
        echo "[build_first] 경로 불일치 감지 → build/ install/ log/ 삭제 중..."
        rm -rf build/ install/ log/
    fi
fi

# 1. 외부 패키지 빌드 (ugv_else, livox 등)
colcon build --packages-select \
    apriltag apriltag_msgs apriltag_ros \
    cartographer \
    costmap_converter_msgs costmap_converter \
    emcl2 \
    explore_lite \
    openslam_gmapping slam_gmapping \
    ldlidar \
    rf2o_laser_odometry \
    robot_pose_publisher \
    teb_msgs teb_local_planner \
    vizanti vizanti_cpp vizanti_demos vizanti_msgs vizanti_server \
    livox_laser_simulation_RO2 livox_ros_driver2 \
    ugv_base_node ugv_interface

# 2. 메인 패키지 빌드
colcon build --packages-select \
    ugv_bringup ugv_chat_ai ugv_description ugv_gazebo \
    ugv_nav ugv_slam ugv_tools ugv_vision ugv_web_app \
    --symlink-install

# 3. 환경 설정 (.bashrc에 추가)
SETUP_BASH="$WS_ROOT/install/setup.bash"

grep -qxF "source /opt/ros/humble/setup.bash" ~/.bashrc \
    || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

grep -qxF "source $SETUP_BASH" ~/.bashrc \
    || echo "source $SETUP_BASH" >> ~/.bashrc

grep -qxF 'eval "$(register-python-argcomplete ros2)"' ~/.bashrc \
    || echo 'eval "$(register-python-argcomplete ros2)"' >> ~/.bashrc

grep -qxF 'eval "$(register-python-argcomplete colcon)"' ~/.bashrc \
    || echo 'eval "$(register-python-argcomplete colcon)"' >> ~/.bashrc

source ~/.bashrc
echo "[build_first] 완료!"