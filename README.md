# FSD_Vehicle: Advanced UGV Autonomous Driving Framework
Professional ROS2 Humble-based development environment for UGV (Unmanned Ground Vehicle) research. Supports high-fidelity Gazebo simulation and multi-platform deployment (x86_64 PC / ARM64 Jetson).

---

## 1. System Specifications
- **Base OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Middleware**: ROS2 Humble Hawksbill
- **Supported Hardware**: 
    - **UGV Models**: UGV ROVER, UGV BEAST, RASP ROVER
    - **Sensors**: Livox Mid-360, LDLiDAR (LD06, LD19, STL27L), Intel Realsense D435i
- **Containers**: Docker Engine with Docker Compose v2.0+

---

## 2. Setup and Installation

### 2.1 Environment Configuration
The project uses a root `.env` file for centralized configuration. Ensure this file exists in the project root before building.

```bash
# Clone repository
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/SeyeongW/FSD_Vehicle.git ugv_ws
cd ugv_ws
```

### 2.2 Docker Image Provisioning
Build images tailored for your target platform. This process installs all system dependencies including OpenCV, PyTorch, and navigation stacks.

```bash
# For PC (Simulation/Development)
# Windows (cmd/powershell):
build_pc
run_pc

# Linux / WSL:
make build_pc
make run_pc
# (or ./build_pc.sh / ./run_pc.sh)

# For physical deployment (Jetson Orin/Nano)
bash docker/run.sh build-jetson
```

### 2.3 로컬 실행 환경 설정 (Docker 없이 직접 실행 시)

Docker를 사용하지 않고 로컬 PC에서 직접 ROS2를 실행할 경우, **워크스페이스 경로**를 설정해야 합니다.  
각 팀원의 유저네임과 디렉토리 구조가 다르기 때문에, `UGV_WS_PATH` 환경변수를 통해 경로를 자동 감지합니다.

#### 자동 설정 (권장)
```bash
# 워크스페이스 루트에서 한 번만 실행
cd ~/ros2_ws/ugv_ws
source setup_local_env.sh
```
> `setup_local_env.sh`는 스크립트 위치를 자동 감지하여 `UGV_WS_PATH`를 설정합니다.  
> `.env` 파일의 기타 환경변수(`UGV_MODEL`, `ROS_DOMAIN_ID` 등)도 함께 로드합니다.

#### 수동 설정
```bash
# 직접 환경변수를 지정할 수도 있습니다
export UGV_WS_PATH=/home/사용자이름/ros2_ws/ugv_ws
```

#### 동작 원리
| 실행 환경 | UGV_WS_PATH 값 | 설정 방법 |
|-----------|---------------|----------|
| **Docker** | `/ros2_ws/ugv_ws` | `docker-compose.yml`에서 자동 설정 |
| **로컬 (자동)** | 스크립트 위치 자동 감지 | `source setup_local_env.sh` |
| **로컬 (수동)** | 직접 지정 | `export UGV_WS_PATH=...` |

> ⚠️ **주의**: `source setup_local_env.sh` 없이 로컬에서 `save_2d_*.sh` 스크립트나 일부 launch 파일을 실행하면 `UGV_WS_PATH` 미설정 에러가 발생합니다.

---

## 3. Workspace Initialization (Inside Container)

After establishing the container environment, initialize the ROS2 workspace.

### 3.1 Initial Build Sequence
```bash
# 1. Enter the container
bash docker/run.sh pc

# 2. Build AprilTag dependency (Native C)
bash build_apriltag.sh

# 3. Full workspace compilation
bash build_first.sh
```

---

## 4. Software Architecture

### 4.1 Core Packages (`src/ugv_main`)
- `ugv_base_node`: Differential kinematics and odometry calculation.
- `ugv_bringup`: Hardware interface for motor control and sensor drivers.
- `ugv_nav`: Navigation2 integration including DWA/TEB local planners.
- `ugv_slam`: SLAM configurations (Gmapping, Cartographer, Rtabmap).
- `ugv_description`: URDF/Xacro models for all supported UGV variants.
- `ugv_gazebo`: Simulation environments and Gazebo-specific plugins.
- `ugv_vision`: Computer vision modules (AprilTag tracking, YOLO).
- `ugv_chat_ai`: LLM-based interaction (Ollama/AI interface).

### 4.2 Dependency Layer (`src/ugv_else`)
Includes integrations for `cartographer`, `teb_local_planner`, `vizanti`, and various LiDAR drivers.

---

## 5. Operational Manual

### 5.1 Teleoperation and Basic Control
```bash
# Export the model before launching
export UGV_MODEL=ugv_rover # Options: ugv_rover, ugv_beast, rasp_rover

# Visualization check (RViz)
ros2 launch ugv_description display.launch.py use_rviz:=true

# Keyboard control
ros2 run ugv_tools keyboard_ctrl

# Joystick control (requires controller connection)
ros2 launch ugv_tools teleop_twist_joy.launch.py
```

### 5.2 Sensor/Hardware Bringup
```bash
# Launch LiDAR and Chassis drivers
# Automatically selects LiDAR model based on environment variables
ros2 launch ugv_bringup bringup_lidar.launch.py
```

### 5.3 Vision-based Interaction (AprilTag)
```bash
# Basic camera bringup
ros2 launch ugv_vision camera.launch.py

# AprilTag target tracking
ros2 run ugv_vision apriltag_track_1
```

### 5.4 Mapping (SLAM)
Support for 2D and 3D mapping using various algorithms.

- **Gmapping (2D)**:
  ```bash
  ros2 launch ugv_slam gmapping.launch.py
  # Save map: ./save_2d_gmapping_map.sh
  ```
- **Cartographer (2D/3D)**:
  ```bash
  ros2 launch ugv_slam cartographer.launch.py
  ```
- **RTAB-Map (3D RGB-D)**:
  ```bash
  ros2 launch ugv_slam rtabmap_rgbd.launch.py
  ```

### 5.5 Navigation2 (Autonomous Driving)
Supports multiple localization and local planning options.
```bash
# Basic Navigation (AMCL + TEB)
ros2 launch ugv_nav nav.launch.py use_localization:=amcl use_localplan:=teb

# Simultaneous SLAM and Navigation
ros2 launch ugv_nav slam_nav.launch.py
```

---

## 6. Gazebo Simulation Environment

For pure simulation development, use the Gazebo-specific launch files.

### 6.1 Simulation Bringup
```bash
# Launch house environment with UGV model
ros2 launch ugv_gazebo bringup.launch.py
```

### 6.2 Simulation Scenarios
- **Autonomous Exploration**: 
  ```bash
  ros2 launch explore_lite explore.launch.py
  ```
- **AI Interaction Test**: 
  ```bash
  ros2 run ugv_chat_ai app
  ```

---

## 7. Technical Notes and Deployment
- **Network Stack**: Configured with `network_mode: host` for low-latency ROS2 discovery.
- **Hardware Mapping**: Serial controllers are mapped to `/dev/ttyTHS1` (Jetson) or `/dev/ttyUSB0` (PC) via `.env`.
- **User Permissions**: Automated `dialout` and `audio` group assignments for immediate hardware access.

---

## 8. Quick Commands Reference (자주 쓰는 명령어)

| Command (명령어) | Description (설명) |
|------------------|------------------|
| `build_pc` | Windows: PC 가상환경 (도커) 이미지 빌드 |
| `run_pc` | Windows: PC 가상환경 (도커) 실행 및 접속 |
| `make build_pc` | Linux/WSL: PC 가상환경 이미지 빌드 |
| `make run_pc` | Linux/WSL: PC 가상환경 실행 및 접속 |
| `bash build_first.sh` | 환경 내 전체 워크스페이스 빌드 (Livox 자동 다운로드 포함) |
| `bash docker/run.sh jetson` | Jetson 배포환경 시작 |
| `bash docker/run.sh stop` | 실행 중인 컨테이너 중지 |

**Maintainer**: SeyeongW  
For architectural details or contribution guidelines, please refer to the project documentation.