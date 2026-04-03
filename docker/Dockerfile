# ============================================================
# UGV Development Environment - PC (x86_64)
# Base: ROS2 Humble + Gazebo + RViz (Full Desktop)
# ============================================================
FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# ── 1. 기본 시스템 도구 ───────────────────────────────────
RUN apt-get update && apt-get install -y \
    git cmake build-essential ninja-build \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    python3-vcstool \
    wget curl vim nano \
    ssh net-tools \
    libssl-dev libusb-1.0-0-dev \
    libopencv-dev python3-opencv \
    # Cartographer 의존성
    liblua5.3-dev libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    libeigen3-dev libceres-dev \
    # TEB Local Planner 의존성
    libg2o-dev \
    && rm -rf /var/lib/apt/lists/*

# ── 2. ROS2 Humble 추가 패키지 ───────────────────────────
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-tf2-tools \
    ros-humble-tf2-ros \
    ros-humble-depthimage-to-laserscan \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-aruco-markers \
    ros-humble-aruco-markers-msgs \
    ros-humble-robot-localization \
    ros-humble-rtabmap-ros \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# ── 3. Python 패키지 ─────────────────────────────────────
RUN pip3 install --no-cache-dir \
    pyserial \
    flask \
    mediapipe \
    requests \
    ultralytics \
    numpy opencv-python-headless

# ── 4. Livox SDK2 (시스템 라이브러리) ─────────────────────
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cd /tmp/Livox-SDK2 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) && make install \
    && ldconfig \
    && rm -rf /tmp/Livox-SDK2

# ── 5. 워크스페이스 경로 생성 ────────────────────────────
RUN mkdir -p /ros2_ws/ugv_ws/src

WORKDIR /ros2_ws/ugv_ws

# ── 6. Entrypoint 설정 ───────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
