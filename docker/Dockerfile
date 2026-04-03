# ============================================================
# UGV Development Environment - PC (x86_64)
# Base: ROS2 Humble + Gazebo + RViz (Full Desktop)
# ============================================================
FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive
ENV PYTHON_VERSION=3.10
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1

# ── 1. 기본 시스템 도구 ───────────────────────────────────
RUN apt-get update && apt-get install -y \
    python3.10 python3.10-dev python3.10-venv \
    python3-pip python3-rosdep python3-colcon-common-extensions \
    python3-vcstool \
    git cmake build-essential ninja-build \
    wget curl vim nano \
    ssh net-tools \
    libssl-dev libusb-1.0-0-dev \
    libopencv-dev python3-opencv \
    # Cartographer 의존성
    liblua5.3-dev libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    libeigen3-dev libceres-dev \
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
    ros-humble-libg2o \
    && rm -rf /var/lib/apt/lists/*

# ── 3. Python 3.10 환경 설정 ─────────────────────────────
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 \
    && update-alternatives --set python3 /usr/bin/python3.10 \
    && python3 -m pip install --upgrade pip setuptools wheel

# 시스템 패키지 sympy 충돌 해결 (distutils 설치본 덮어쓰기)
RUN pip3 install --no-cache-dir --ignore-installed sympy

# 기본 패키지
RUN pip3 install --no-cache-dir \
    pyserial==3.5 \
    flask==3.0.3 \
    requests==2.31.0 \
    numpy==1.26.4

# mediapipe (opencv-contrib-python 포함)
RUN pip3 install --no-cache-dir mediapipe==0.10.14

# PyTorch CPU 버전 (PC 개발용 - CUDA 버전은 10배 더 큼)
RUN pip3 install --no-cache-dir \
    torch torchvision \
    --index-url https://download.pytorch.org/whl/cpu

# YOLO (mediapipe가 설치한 opencv 재사용)
RUN pip3 install --no-cache-dir ultralytics

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
