# ugv_hack: Advanced UGV Autonomous Driving Framework
Professional ROS2 Humble-based development environment for UGV (Unmanned Ground Vehicle) research, focused on high-fidelity Gazebo simulation.

---

## 1. System Specifications
- **Base OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Middleware**: ROS2 Humble Hawksbill
- **Simulation**: Gazebo (Classic)
- **Supported Hardware**:
    - **UGV Models**: UGV ROVER, UGV BEAST, RASP ROVER
    - **Sensors**: Livox Mid-360, LDLiDAR (LD06, LD19, STL27L), Intel Realsense D435i

---

## 2. Setup and Installation

### 2.1 Prerequisites
Install ROS2 Humble (desktop-full) on Ubuntu 22.04 and make sure Gazebo is available.
See the [official ROS2 Humble install guide](https://docs.ros.org/en/humble/Installation.html).

### 2.2 Clone the Repository
```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/SeyeongW/FSD_Vehicle.git ugv_hack
cd ugv_hack
```

### 2.3 로컬 환경 설정

로컬 PC에서 직접 ROS2를 실행하기 위해 **워크스페이스 경로**를 설정합니다.
각 팀원의 유저네임과 디렉토리 구조가 다르기 때문에, `UGV_WS_PATH` 환경변수를 통해 경로를 자동 감지합니다.

#### 자동 설정 (권장)
```bash
# 워크스페이스 루트에서 한 번만 실행
cd ~/ros2_ws/ugv_hack
source setup_local_env.sh
```
> `setup_local_env.sh`는 스크립트 위치를 자동 감지하여 `UGV_WS_PATH`를 설정합니다.
> `.env` 파일의 기타 환경변수(`UGV_MODEL`, `ROS_DOMAIN_ID` 등)도 함께 로드합니다.

#### 수동 설정
```bash
# 직접 환경변수를 지정할 수도 있습니다
export UGV_WS_PATH=/home/사용자이름/ros2_ws/ugv_hack
```

> ⚠️ **주의**: `source setup_local_env.sh` 없이 `save_2d_*.sh` 스크립트나 일부 launch 파일을 실행하면 `UGV_WS_PATH` 미설정 에러가 발생합니다.

---

## 3. Workspace Initialization

### 3.1 Initial Build Sequence
```bash
# Full workspace compilation (Livox SDK/driver auto-downloaded)
bash build_first.sh
```

`build_first.sh`는 최초 1회 실행하여 외부 SDK(Livox)를 자동으로 클론·빌드하고 전체 워크스페이스를 컴파일합니다.

### 3.2 Incremental Rebuild
소스코드만 수정한 경우에는 메인 패키지만 빠르게 재빌드합니다.
```bash
bash build_common.sh
```

---

## 4. Software Architecture

### 4.1 Core Packages (`src/ugv_main`)
- `ugv_base_node`: Differential kinematics and odometry calculation.
- `ugv_interface`: Shared message/interface definitions.
- `ugv_description`: URDF/Xacro models for all supported UGV variants.
- `ugv_gazebo`: gz (Harmonic) simulation — worlds, models, spawn, and Nav2 launches.

### 4.2 Dependency Layer (`src/ugv_else`)
Nav-stack dependencies: `teb_local_planner` + `costmap_converter` (local planner),
`emcl2` (localization), `robot_pose_publisher`. Sensor drivers: `ldlidar`.
Livox stack (`livox_ros_driver2`, `livox_laser_simulation_RO2`, `Livox-SDK2`) is
auto-cloned by `build_first.sh`.

### 4.3 Gimbal Camera (`gimbal_camera/`)
Standalone 3-axis gimbal camera for gz Harmonic (no `colcon build` needed).
See `gimbal_camera/README.md`.

---

## 5. Operational Manual

The workflow is **simulation-only**: spawn the rover in gz and drive it with Nav2
(localization on a prebuilt map — no SLAM).

### 5.1 Simulation Bringup
```bash
# Export the model before launching
export UGV_MODEL=ugv_rover # Options: ugv_rover, ugv_beast, rasp_rover

# Spawn the rover into the gz world (+ ros_gz bridge, birds)
ros2 launch ugv_gazebo bringup.launch.py

# Model-only visualization check (RViz)
ros2 launch ugv_description display.launch.py use_rviz:=true
```

### 5.2 Navigation2 (Autonomous Driving, no SLAM)
Localization runs on the prebuilt map in `ugv_gazebo/maps`.
```bash
# AMCL localization + TEB local planner (defaults)
ros2 launch ugv_gazebo nav/nav.launch.py use_localization:=amcl use_localplan:=teb

# Options: use_localization:=amcl|emcl , use_localplan:=teb|dwa
```

### 5.3 Gimbal Camera
```bash
ros2 launch gimbal_camera gimbal_camera.launch.py
```

---

## 6. Technical Notes
- **Network Stack**: ROS2 DDS uses UDP multicast for node discovery; keep all nodes on the same `ROS_DOMAIN_ID`.
- **Hardware Mapping**: Serial controllers default to `/dev/ttyUSB0` (PC) via `.env`.

---

## 7. Quick Commands Reference (자주 쓰는 명령어)

| Command (명령어) | Description (설명) |
|------------------|------------------|
| `source setup_local_env.sh` | 워크스페이스 경로 자동 설정 |
| `bash build_first.sh` | 전체 워크스페이스 빌드 (Livox SDK 자동 다운로드 포함) |
| `bash build_common.sh` | 메인 패키지만 빠른 재빌드 |
| `ros2 launch ugv_gazebo bringup.launch.py` | gz 시뮬레이션에 로버 스폰 |
| `ros2 launch ugv_gazebo nav/nav.launch.py` | Nav2 자율주행 (SLAM 미사용) |
| `ros2 launch gimbal_camera gimbal_camera.launch.py` | 짐벌 카메라 |

**Maintainer**: SeyeongW
For architectural details or contribution guidelines, please refer to the project documentation.
