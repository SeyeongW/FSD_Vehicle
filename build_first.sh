#!/bin/bash
# 전체 빌드 스크립트 (최초 1회 실행)

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[build_first] 워크스페이스 루트: $WS_ROOT"
cd "$WS_ROOT"

# ROS2 환경 소싱 (필수)
source /opt/ros/humble/setup.bash

# RMW 설정 (dustynv Jetson 이미지는 cyclonedds만 포함)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 기존 빌드 캐시가 다른 경로에서 만들어졌으면 자동 삭제
if [ -f "build/apriltag/CMakeCache.txt" ]; then
    CACHED_PATH=$(grep "CMAKE_CACHEFILE_DIR" build/apriltag/CMakeCache.txt 2>/dev/null | cut -d= -f2)
    if [ "$CACHED_PATH" != "$WS_ROOT/build/apriltag" ]; then
        echo "[build_first] 경로 불일치 감지 → build/ install/ log/ 삭제 중..."
        rm -rf build/ install/ log/
    fi
fi

# 0. 외부 SDK 및 드라이버 패키지 다운로드 및 설치 (Jetson 하드웨어 전용)
echo "[build_first] 확인: SDK 및 드라이버 소스코드 검사..."
cd "$WS_ROOT/src"

if [ ! -d "Livox-SDK2" ]; then
    echo ">> Cloning Livox-SDK2..."
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd Livox-SDK2 && mkdir -p build && cd build
    cmake .. && make -j$(nproc) && sudo make install
    cd "$WS_ROOT/src"
fi

if [ ! -d "livox_ros_driver2" ] || [ -z "$(ls -A livox_ros_driver2 2>/dev/null)" ]; then
    echo ">> Cloning livox_ros_driver2..."
    rm -rf livox_ros_driver2
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
fi

cd "$WS_ROOT"

# 1. 외부 패키지 빌드 (ugv_else, livox 드라이버 — Jetson 전용)
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
    livox_ros_driver2 \
    ugv_base_node ugv_interface \
    --cmake-args -DHUMBLE_ROS=humble

# 2. 메인 패키지 빌드 (Jetson 하드웨어 전용 — 시뮬레이션 제외)
colcon build --packages-select \
    ugv_bringup ugv_chat_ai ugv_description \
    ugv_nav ugv_slam ugv_tools ugv_vision ugv_web_app ugv_lidar_detection \
    --symlink-install

# 3. Python 패키지 빌드 (--symlink-install 제외 — setuptools 호환성 문제)
colcon build --packages-select \
    pcd_cluster_pkg pcd_to_scan_pkg plane_fit_pkg

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