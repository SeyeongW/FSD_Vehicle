# FSD_Vehicle — Jetson Hardware Guide

> **Branch**: `jetson` | **Target**: Jetson Orin Nano Super (ARM64) | **ROS2**: Humble

---

## 목차

1. [시스템 사양](#1-시스템-사양)
2. [최초 환경 구성](#2-최초-환경-구성)
3. [Docker 빌드 & 실행](#3-docker-빌드--실행)
4. [워크스페이스 빌드](#4-워크스페이스-빌드)
5. [로버 구동 (하드웨어 Bringup)](#5-로버-구동-하드웨어-bringup)
6. [원격 조종](#6-원격-조종)
7. [센서 단독 실행](#7-센서-단독-실행)
8. [SLAM (지도 생성)](#8-slam-지도-생성)
9. [Navigation2 (자율주행)](#9-navigation2-자율주행)
10. [비전 모듈](#10-비전-모듈)
11. [라이다 탐지 (Jetson 전용)](#11-라이다-탐지-jetson-전용)
12. [AI 인터랙션](#12-ai-인터랙션)
13. [유틸리티 및 진단](#13-유틸리티-및-진단)
14. [빠른 명령어 레퍼런스](#14-빠른-명령어-레퍼런스)

---

## 1. 시스템 사양

| 항목 | 내용 |
|------|------|
| **하드웨어** | NVIDIA Jetson Orin Nano Super |
| **OS** | Ubuntu 22.04 LTS (JetPack 6.x, L4T r36.4.0) |
| **미들웨어** | ROS2 Humble Hawksbill |
| **컨테이너** | Docker Engine + Docker Compose v2.0+ |
| **지원 로봇** | UGV ROVER / UGV BEAST / RASP ROVER |
| **지원 센서** | Livox Mid-360, LDLiDAR (LD06 / LD19 / STL27L), Intel RealSense D435i, OAK-D Lite |
| **시리얼 포트** | `/dev/ttyTHS1` (Jetson 기본) |
| **카메라 장치** | `/dev/video0` |

---

## 2. 최초 환경 구성

### 2.1 저장소 클론

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone -b jetson https://github.com/SeyeongW/FSD_Vehicle.git ugv_ws
cd ugv_ws
```

### 2.2 `.env` 설정 확인

`.env` 파일에서 로봇 모델과 장치 경로를 확인·수정합니다.

```bash
cat .env
```

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `UGV_MODEL` | `ugv_rover` | 로봇 모델 (`ugv_rover` / `ugv_beast` / `rasp_rover`) |
| `SERIAL_PORT_JETSON` | `/dev/ttyTHS1` | 모터 컨트롤러 시리얼 포트 |
| `VIDEO_DEVICE` | `/dev/video0` | 카메라 장치 |
| `ROS_DOMAIN_ID` | `0` | ROS2 네트워크 도메인 |

```bash
# 예: 로봇 모델 변경
sed -i 's/UGV_MODEL=.*/UGV_MODEL=ugv_beast/' .env
```

---

## 3. Docker 빌드 & 실행

### 3.1 Jetson 이미지 빌드 (최초 1회)

```bash
# Makefile 사용
make build_jetson

# 또는 직접 실행
bash docker/run.sh build-jetson
```

> 빌드 시 Livox SDK2, PyTorch (CUDA), Ultralytics YOLO 등이 자동 설치됩니다. 약 15~30분 소요.

### 3.2 컨테이너 진입

```bash
# Makefile 사용
make run_jetson

# 또는 직접 실행
bash docker/run.sh jetson
```

### 3.3 컨테이너 중지

```bash
bash docker/run.sh stop
# 또는
make stop
```

### 3.4 컨테이너 재접속 (이미 실행 중일 때)

```bash
docker exec -it fsd_dev_jetson bash
```

---

## 4. 워크스페이스 빌드

> **컨테이너 내부**에서 실행합니다.

### 4.1 최초 빌드 (최초 1회)

Livox-SDK2, livox_ros_driver2 자동 클론 + 전체 패키지 빌드

```bash
# 1. AprilTag C 라이브러리 빌드 (최초 1회)
bash build_apriltag.sh

# 2. 전체 워크스페이스 빌드
bash build_first.sh
```

### 4.2 코드 수정 후 재빌드

```bash
bash build_common.sh
```

### 4.3 특정 패키지만 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select <패키지명> --symlink-install
source install/setup.bash
```

---

## 5. 로버 구동 (하드웨어 Bringup)

> 이 단계에서 모터, 라이다, 오도메트리가 모두 기동됩니다.

### 5.1 기본 Bringup (LDLiDAR + 오도메트리)

```bash
export UGV_MODEL=ugv_rover   # ugv_rover / ugv_beast / rasp_rover
ros2 launch ugv_bringup bringup_lidar.launch.py
```

| 옵션 | 기본값 | 설명 |
|------|--------|------|
| `use_rviz` | `false` | RViz2 실행 여부 |
| `pub_odom_tf` | `true` | odom → base_footprint TF 퍼블리시 여부 |

```bash
# RViz 포함 기동
ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true
```

### 5.2 IMU + EKF Bringup (융합 오도메트리)

```bash
ros2 launch ugv_bringup bringup_imu_ekf.launch.py
```

IMU 상보 필터(complementary filter) + EKF(robot_localization)를 통해 오도메트리를 융합합니다.

```bash
# RViz 포함
ros2 launch ugv_bringup bringup_imu_ekf.launch.py use_rviz:=true
```

### 5.3 로봇 모델 시각화만 (하드웨어 없이)

```bash
ros2 launch ugv_description display.launch.py use_rviz:=true
```

---

## 6. 원격 조종

> Bringup이 실행 중인 상태에서 **별도 터미널**로 실행합니다.

### 6.1 키보드 조종

```bash
ros2 run ugv_tools keyboard_ctrl
```

| 키 | 동작 |
|----|------|
| `W` / `S` | 전진 / 후진 |
| `A` / `D` | 좌회전 / 우회전 |
| `Space` | 정지 |
| `Q` | 종료 |

### 6.2 조이스틱 조종 (USB 컨트롤러 연결 필요)

```bash
ros2 launch ugv_tools teleop_twist_joy.launch.py
```

### 6.3 행동 제어 (Behavior Control)

```bash
ros2 run ugv_tools behavior_ctrl
```

---

## 7. 센서 단독 실행

### 7.1 LDLiDAR 단독 실행

```bash
ros2 launch ldlidar ldlidar.launch.py
```

### 7.2 Livox Mid-360 단독 실행

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 7.3 Intel RealSense D435i

```bash
ros2 launch realsense2_camera rs_launch.py
```

### 7.4 OAK-D Lite 카메라

```bash
ros2 launch ugv_vision oak_d_lite.launch.py
```

### 7.5 일반 USB 카메라

```bash
ros2 launch ugv_vision camera.launch.py
```

### 7.6 라이다 오도메트리 (rf2o)

```bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py
```

---

## 8. SLAM (지도 생성)

> **Bringup 없이** 단독으로 실행합니다. 각 launch 파일 내부에서 bringup을 포함합니다.

### 8.1 Gmapping (2D LiDAR SLAM)

```bash
ros2 launch ugv_slam gmapping.launch.py
```

```bash
# 지도 저장
bash save_2d_gmapping_map.sh
```

### 8.2 Cartographer (2D/3D LiDAR SLAM)

```bash
ros2 launch ugv_slam cartographer.launch.py
```

```bash
# 지도 저장
bash save_2d_cartographer_map.sh
```

### 8.3 RTAB-Map (3D RGB-D SLAM, OAK-D Lite 필요)

```bash
# SLAM 모드 (새 지도 생성)
ros2 launch ugv_slam rtabmap_rgbd.launch.py

# 로컬라이제이션 모드 (기존 지도 사용)
ros2 launch ugv_slam rtabmap_rgbd.launch.py localization:=true
```

---

## 9. Navigation2 (자율주행)

> 사전에 SLAM으로 저장한 지도(`map.yaml`)가 필요합니다.

### 9.1 기본 네비게이션 (AMCL + TEB)

```bash
ros2 launch ugv_nav nav.launch.py
```

| 옵션 | 기본값 | 선택지 | 설명 |
|------|--------|--------|------|
| `use_localization` | `amcl` | `amcl` / `emcl` / `cartographer` | 위치추정 방식 |
| `use_localplan` | `teb` | `teb` / `dwa` | 로컬 플래너 |
| `map` | `ugv_nav/maps/map.yaml` | 임의 경로 | 지도 파일 경로 |
| `use_rviz` | `false` | `true` / `false` | RViz2 실행 여부 |

```bash
# 조합 예시
ros2 launch ugv_nav nav.launch.py \
    use_localization:=amcl \
    use_localplan:=teb \
    map:=/ros2_ws/ugv_ws/maps/my_map.yaml \
    use_rviz:=true
```

### 9.2 SLAM + 네비게이션 동시 실행

```bash
ros2 launch ugv_nav slam_nav.launch.py
```

### 9.3 RTAB-Map + 네비게이션

```bash
ros2 launch ugv_nav nav_rtabmap.launch.py
```

### 9.4 RTAB-Map 로컬라이제이션 모드

```bash
ros2 launch ugv_nav rtabmap_localization_launch.py
```

### 9.5 자율 탐색 (Explore Lite)

```bash
ros2 launch explore_lite explore.launch.py
```

---

## 10. 비전 모듈

### 10.1 AprilTag 트래킹

```bash
# 카메라 + AprilTag 감지 런치
ros2 launch ugv_vision apriltag_track.launch.py

# 또는 노드 단독 실행
ros2 run ugv_vision apriltag_track_1
```

### 10.2 스테레오 카메라 (Depth 추정)

```bash
ros2 launch ugv_vision stereo.launch.py
```

---

## 11. 라이다 탐지 (Jetson 전용)

Jetson CUDA를 활용한 포인트클라우드 기반 장애물 탐지 패키지입니다.

### 11.1 라이다 탐지 노드

```bash
ros2 launch ugv_lidar_detection lidar_detection.launch.py
```

### 11.2 포인트클라우드 처리 노드

```bash
# PCD 클러스터링
ros2 run pcd_cluster_pkg pcd_cluster_node

# PCD → LaserScan 변환
ros2 run pcd_to_scan_pkg pcd_to_scan_node

# 지면 제거 (Plane Fit)
ros2 run plane_fit_pkg plane_fit_node
```

---

## 12. AI 인터랙션

Ollama LLM 기반 챗봇 인터페이스입니다.

```bash
ros2 run ugv_chat_ai app
```

> 웹 브라우저에서 `http://<젯슨_IP>:5000` 으로 접속합니다.

---

## 13. 유틸리티 및 진단

### 13.1 환경변수 설정 (Docker 없이 로컬 실행 시)

```bash
source setup_local_env.sh
```

### 13.2 ROS2 토픽 확인

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic hz /scan         # 라이다 주기 확인
```

### 13.3 TF 트리 확인

```bash
ros2 run tf2_tools view_frames
```

### 13.4 RViz2 단독 실행

```bash
rviz2
```

### 13.5 시리얼 포트 권한 확인

```bash
ls -la /dev/ttyTHS1
sudo chmod 666 /dev/ttyTHS1
```

### 13.6 컨테이너 로그 확인

```bash
docker logs fsd_dev_jetson -f
```

### 13.7 GPU 상태 확인 (Jetson)

```bash
tegrastats
# 또는
watch -n 1 tegrastats
```

---

## 14. 빠른 명령어 레퍼런스

### Docker 관련

| 명령어 | 설명 |
|--------|------|
| `make build_jetson` | Jetson Docker 이미지 빌드 |
| `make run_jetson` | 컨테이너 실행 및 진입 |
| `make stop` | 컨테이너 중지 |
| `bash docker/run.sh jetson` | 컨테이너 실행 (직접 실행) |
| `bash docker/run.sh build-jetson` | 이미지 빌드 (직접 실행) |
| `bash docker/run.sh stop` | 컨테이너 중지 (직접 실행) |
| `docker exec -it fsd_dev_jetson bash` | 실행 중 컨테이너 재접속 |

### 빌드 관련

| 명령어 | 설명 |
|--------|------|
| `bash build_apriltag.sh` | AprilTag C 라이브러리 빌드 (최초 1회) |
| `bash build_first.sh` | 전체 워크스페이스 최초 빌드 |
| `bash build_common.sh` | 코드 수정 후 재빌드 |

### 로버 구동

| 명령어 | 설명 |
|--------|------|
| `ros2 launch ugv_bringup bringup_lidar.launch.py` | 기본 Bringup (LiDAR + 오도메트리) |
| `ros2 launch ugv_bringup bringup_imu_ekf.launch.py` | IMU + EKF Bringup |
| `ros2 run ugv_tools keyboard_ctrl` | 키보드 조종 |
| `ros2 launch ugv_tools teleop_twist_joy.launch.py` | 조이스틱 조종 |
| `ros2 run ugv_tools behavior_ctrl` | 행동 제어 |

### SLAM

| 명령어 | 설명 |
|--------|------|
| `ros2 launch ugv_slam gmapping.launch.py` | Gmapping 2D SLAM |
| `ros2 launch ugv_slam cartographer.launch.py` | Cartographer SLAM |
| `ros2 launch ugv_slam rtabmap_rgbd.launch.py` | RTAB-Map 3D SLAM |
| `bash save_2d_gmapping_map.sh` | Gmapping 지도 저장 |
| `bash save_2d_cartographer_map.sh` | Cartographer 지도 저장 |

### 자율주행

| 명령어 | 설명 |
|--------|------|
| `ros2 launch ugv_nav nav.launch.py` | Navigation2 (AMCL + TEB) |
| `ros2 launch ugv_nav slam_nav.launch.py` | SLAM + 네비게이션 동시 |
| `ros2 launch ugv_nav nav_rtabmap.launch.py` | RTAB-Map 네비게이션 |
| `ros2 launch explore_lite explore.launch.py` | 자율 탐색 |

### 센서

| 명령어 | 설명 |
|--------|------|
| `ros2 launch ldlidar ldlidar.launch.py` | LDLiDAR 단독 실행 |
| `ros2 launch livox_ros_driver2 msg_MID360_launch.py` | Livox Mid-360 단독 실행 |
| `ros2 launch realsense2_camera rs_launch.py` | RealSense D435i |
| `ros2 launch ugv_vision oak_d_lite.launch.py` | OAK-D Lite 카메라 |
| `ros2 launch ugv_vision camera.launch.py` | USB 카메라 |

### 라이다 탐지 (Jetson)

| 명령어 | 설명 |
|--------|------|
| `ros2 launch ugv_lidar_detection lidar_detection.launch.py` | 라이다 탐지 런치 |
| `ros2 run pcd_cluster_pkg pcd_cluster_node` | PCD 클러스터링 |
| `ros2 run pcd_to_scan_pkg pcd_to_scan_node` | PCD → LaserScan 변환 |
| `ros2 run plane_fit_pkg plane_fit_node` | 지면 제거 |

---

**Maintainer**: SeyeongW  
**Branch**: `jetson` | Gazebo/PC 시뮬레이션은 `main_s` 브랜치를 참고하세요.