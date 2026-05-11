# FSD_Vehicle 기술 연구일지

**프로젝트**: FSD_Vehicle — UGV 자율주행 플랫폼  
**작성자**: SeyeongW  
**작성일**: 2026-05-11  
**대상 플랫폼**: UGV ROVER / UGV BEAST / RASP ROVER  
**미들웨어**: ROS2 Humble Hawksbill (Ubuntu 22.04 LTS)

---

## 목차

1. [개발 환경 구성](#1-개발-환경-구성)
2. [Docker 환경 설계](#2-docker-환경-설계)
3. [Unitree 4D LiDAR 탐지 기술](#3-unitree-4d-lidar-탐지-기술)
4. [시스템 통합 구조](#4-시스템-통합-구조)

---

## 1. 개발 환경 구성

### 1.1 배경 및 목적

UGV 자율주행 플랫폼을 개발하면서 가장 먼저 맞닥뜨린 문제는 **환경 일관성**이었다. ROS2 패키지는 의존성이 복잡하고, 팀원마다 PC 환경이 달라 "내 환경에서는 되는데 네 환경에서는 안 된다"는 상황이 빈번했다. 또한 실제 배포 대상인 Jetson Orin과 개발용 x86 PC는 아키텍처가 달라, 빌드 환경 자체를 분리해서 관리할 필요가 있었다.

이를 해결하기 위해 다음 세 가지 원칙을 세웠다.

- **재현 가능성**: Docker 이미지로 환경을 코드화하여 누구나 동일한 환경을 구축할 수 있게 한다
- **멀티 플랫폼 지원**: x86_64 PC용 이미지와 ARM64 Jetson용 이미지를 별도로 관리한다
- **중앙 집중 설정**: 하드웨어 모델명, 시리얼 포트 등 배포 환경마다 달라지는 값들은 `.env` 파일 하나에서 관리한다

---

### 1.2 환경 변수 설계 (`.env`)

모든 환경 의존적인 설정값은 프로젝트 루트의 `.env` 파일에 집중시켰다. Docker Compose가 이 파일을 자동으로 읽어 컨테이너에 전달하고, 로컬 실행 시에는 `setup_local_env.sh`가 동일한 파일을 로드한다.

```bash
# 주요 설정값
COMPOSE_PROJECT_NAME=fsd_vehicle
IMAGE_NAME=fsd-vehicle
CONTAINER_NAME=fsd_dev

UGV_MODEL=ugv_rover          # ugv_rover | ugv_beast | rasp_rover
SERIAL_PORT_JETSON=/dev/ttyTHS1
SERIAL_PORT_PC=/dev/ttyUSB0
VIDEO_DEVICE=/dev/video0
ROS_DOMAIN_ID=0
```

이렇게 하면 새로운 하드웨어 모델을 추가하거나 시리얼 포트가 바뀌더라도 `.env` 파일 한 줄만 수정하면 된다. Launch 파일이나 소스코드를 직접 수정할 필요가 없다.

**로컬 실행 지원**: Docker 없이 직접 ROS2를 실행하는 경우를 위해 `setup_local_env.sh`를 별도로 작성했다. 이 스크립트는 자신의 위치를 기반으로 `UGV_WS_PATH`를 자동 감지하기 때문에, 팀원마다 워크스페이스 경로가 달라도 문제없이 동작한다.

```bash
source setup_local_env.sh   # 워크스페이스 루트에서 1회 실행
```

---

### 1.3 빌드 시스템

ROS2 워크스페이스는 colcon 빌드 시스템을 사용하는데, 외부 SDK까지 포함하면 처음 빌드 순서가 복잡해진다. 이 문제를 해결하기 위해 빌드 스크립트를 용도별로 분리했다.

#### `build_first.sh` — 최초 1회 전체 빌드

처음 환경을 세팅할 때 실행하는 스크립트다. 외부 SDK를 자동으로 클론 및 설치한 뒤 전체 패키지를 빌드한다.

```
실행 순서:
1. Livox-SDK2 클론 → CMake 빌드 → 시스템 라이브러리 설치
2. unitree_lidar_sdk 클론 → CMake 빌드 → 시스템 라이브러리 설치
3. livox_ros_driver2, livox_laser_simulation_RO2 클론
4. 외부 의존 패키지 colcon 빌드 (apriltag, cartographer, teb, slam 등)
5. 메인 패키지 빌드 (--symlink-install)
6. ~/.bashrc에 ROS2 환경 소싱 자동 추가
```

SDK 디렉토리가 이미 존재하는지 먼저 확인하기 때문에, 스크립트를 여러 번 실행해도 중복 다운로드가 발생하지 않는다. 또한 CMake 캐시 경로 불일치가 감지되면 `build/`, `install/`, `log/` 디렉토리를 자동으로 정리하여 빌드 오염을 방지한다.

#### `build_common.sh` — 코드 수정 후 빠른 재빌드

외부 SDK는 변경이 없으므로 재빌드할 필요가 없다. 소스코드만 수정한 경우에는 이 스크립트로 메인 패키지만 빠르게 재빌드한다.

#### `build_apriltag.sh` — AprilTag C 라이브러리 별도 빌드

AprilTag는 C 라이브러리를 먼저 시스템에 설치해야 ROS2 wrapper가 빌드된다. 이 절차가 다른 패키지 빌드와 순서가 엄격하여 별도 스크립트로 분리했다.

#### 크로스 플랫폼 지원

Windows 환경에서도 동일한 명령을 사용할 수 있도록 `.bat` 파일을 함께 제공했다.

| 명령 | Linux/macOS | Windows |
|------|-------------|---------|
| 이미지 빌드 | `make build_pc` | `build_pc.bat` |
| 컨테이너 실행 | `make run_pc` | `run_pc.bat` |

---

## 2. Docker 환경 설계

### 2.1 이중 이미지 전략 (Dual-Image Strategy)

개발 단계와 배포 단계의 요구사항이 다르기 때문에, 이미지를 두 개로 분리하는 전략을 선택했다.

```
개발 (PC / x86_64)                 배포 (Jetson / ARM64)
─────────────────────────────      ─────────────────────────────
osrf/ros:humble-desktop-full  →   dustynv/ros:humble-desktop-l4t-r36.4.0
PyTorch CPU (개발/시뮬레이션)       PyTorch + CUDA (기본 포함)
Gazebo 시뮬레이터 포함              Gazebo 제외 (실물 로봇 전용)
OpenCV, MediaPipe 포함              Livox SDK2 이미지 내 빌드
```

PC 이미지는 시뮬레이션 환경(Gazebo)과 GUI 도구가 포함되어 개발과 디버깅에 최적화되어 있다. Jetson 이미지는 실제 하드웨어 배포를 위해 CUDA를 활용한 AI 추론에 집중하고, 불필요한 Gazebo를 제외하여 이미지 크기와 메모리 사용량을 줄였다.

---

### 2.2 PC Dockerfile (`docker/Dockerfile`)

베이스 이미지로 `osrf/ros:humble-desktop-full`을 선택했다. ROS2 Humble의 공식 이미지로, 신뢰성이 높고 RViz 등 GUI 도구가 포함되어 있다.

**레이어 구성 설계**

Docker 이미지는 레이어 캐시를 활용하기 위해 변경 빈도가 낮은 것부터 높은 것 순으로 레이어를 구성했다.

```dockerfile
# 레이어 1: 기본 시스템 도구 (가장 안정적)
RUN apt-get install -y \
    python3.10 python3.10-dev cmake build-essential ninja-build \
    libssl-dev libusb-1.0-0-dev libopencv-dev \
    # Cartographer 의존성
    liblua5.3-dev libgoogle-glog-dev libeigen3-dev libceres-dev ...

# 레이어 2: ROS2 Humble 확장 패키지
RUN apt-get install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-cartographer-ros \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-rtabmap-ros ros-humble-realsense2-camera \
    ros-humble-robot-localization ros-humble-pcl-ros ...

# 레이어 3: Python ML 패키지 (버전 고정)
RUN pip3 install pyserial==3.5 flask==3.0.3 numpy==1.26.4 scikit-learn
RUN pip3 install mediapipe==0.10.14
RUN pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install ultralytics
```

NumPy 버전을 `1.26.4`로 고정한 것은 scikit-learn과의 ABI 호환성을 위해서다. PyTorch는 PC 개발 환경에서 CUDA 불필요하므로 CPU 버전을 설치하여 이미지 크기를 크게 줄였다 (GPU 버전 대비 약 4GB 절약).

**sympy 충돌 처리**

ROS2 기본 이미지의 시스템 sympy와 pip sympy 간 distutils 충돌이 발생하는 문제가 있었다. `--ignore-installed` 플래그로 pip 버전을 강제 설치하여 해결했다.

---

### 2.3 Jetson Dockerfile (`docker/Dockerfile.jetson`)

Jetson 환경에서는 Unitree 4D LiDAR SDK를 이미지 빌드 시점에 컴파일해서 포함시켰다. PC 환경에서는 `build_first.sh`가 런타임에 SDK를 다운로드하지만, Jetson은 네트워크 환경이 불안정한 현장 배포를 고려해 이미지 자체에 포함시키는 방식을 택했다.

```dockerfile
# Livox SDK2 이미지 내 컴파일
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cd /tmp/Livox-SDK2 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release \
    && make -j$(nproc) && make install \
    && ldconfig \
    && rm -rf /tmp/Livox-SDK2    # 빌드 소스 제거로 이미지 크기 절약
```

**ARM64 APT 소스 재구성 문제**

`dustynv` 베이스 이미지는 L4T(Linux for Tegra) 전용 APT 소스만 포함하고 있어, 일반 Ubuntu 패키지 설치 시 `404 Not Found` 오류가 발생했다. ARM64용 Ubuntu 포트 저장소를 수동으로 추가하고 ROS2 GPG 키를 재등록하는 방식으로 해결했다.

```dockerfile
RUN rm -f /etc/apt/sources.list.d/ros2* && \
    echo "deb http://ports.ubuntu.com/ubuntu-ports jammy universe multiverse" \
    >> /etc/apt/sources.list
```

---

### 2.4 Docker Compose 설정

#### PC 환경 (`docker-compose.yml`)

개발 환경에서는 RViz와 Gazebo 등 GUI 애플리케이션을 사용해야 하기 때문에 X11 소켓을 컨테이너에 마운트했다.

```yaml
services:
  fsd-dev:
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw   # X11 GUI 지원
      - .:/ros2_ws/ugv_ws                  # 소스코드 실시간 반영
      - ${HOME}/.gazebo:/root/.gazebo      # Gazebo 모델 캐시
    network_mode: host      # ROS2 DDS 멀티캐스트를 위해 host 네트워크 사용
    privileged: true        # 하드웨어 직접 접근
```

`network_mode: host`를 사용한 이유는 ROS2의 DDS(Data Distribution Service) 통신이 UDP 멀티캐스트를 사용하기 때문이다. NAT 없이 호스트 네트워크를 직접 공유해야 노드 간 자동 탐색이 정상 동작한다.

소스코드 디렉토리를 bind mount로 연결했기 때문에, 호스트에서 코드를 수정하면 컨테이너 내부에서 즉시 반영된다. `--symlink-install`로 빌드된 패키지는 재빌드 없이 변경사항이 적용된다.

#### Jetson 배포 환경 (`docker-compose.jetson.yml`)

실제 하드웨어가 연결되는 Jetson에서는 센서 디바이스 파일 접근이 핵심이다.

```yaml
services:
  fsd-jetson:
    runtime: nvidia                           # CUDA 활성화
    restart: unless-stopped                   # 재부팅 시 자동 시작
    volumes:
      - /dev:/dev                             # 전체 디바이스 마운트
      - /run/udev:/run/udev:ro                # udev 핫플러그 감지
    devices:
      - ${SERIAL_PORT_JETSON}:${SERIAL_PORT_JETSON}   # 직렬 통신 (모터 컨트롤러)
      - ${VIDEO_DEVICE}:${VIDEO_DEVICE}               # 카메라
      - /dev/snd:/dev/snd                             # 오디오
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all,compute,video
```

`restart: unless-stopped` 정책으로 Jetson이 재부팅되면 컨테이너가 자동으로 재시작되도록 했다. 이를 통해 현장 배포 후 전원이 꺼졌다 켜지더라도 별도 조작 없이 시스템이 복구된다.

---

### 2.5 컨테이너 관리 스크립트

매번 긴 `docker compose` 명령을 입력하는 불편함을 줄이기 위해 `docker/run.sh` 래퍼 스크립트를 작성했다.

```bash
bash docker/run.sh pc           # PC 컨테이너 시작 및 접속
bash docker/run.sh jetson       # Jetson 컨테이너 시작 및 접속
bash docker/run.sh stop         # 모든 컨테이너 정지
bash docker/run.sh build-pc     # PC 이미지만 빌드
bash docker/run.sh build-jetson # Jetson 이미지만 빌드
```

스크립트는 실행 전 X11 xhost 설정을 자동으로 처리하고, 디바이스 파일의 존재 여부를 확인한 뒤 컨테이너를 시작한다.

`docker/entrypoint.sh`는 컨테이너 진입 시 ROS2 환경을 자동으로 소싱하고, 워크스페이스가 아직 빌드되지 않은 경우 빌드 방법을 안내하는 메시지를 출력한다.

---

## 3. Unitree 4D LiDAR 탐지 기술

### 3.1 센서 선정 배경

UGV의 주변 환경을 인식하기 위한 센서를 검토하면서 **Unitree 4D LiDAR**를 선택했다. 기존 2D LiDAR는 단일 평면만 스캔하기 때문에 높이 정보가 없어 장애물 높이를 구분할 수 없다는 한계가 있었다. Unitree의 4D LiDAR는 공간 3차원(X, Y, Z) 좌표에 반사 강도(Intensity)까지 포함한 4차원 데이터를 실시간으로 제공한다. 이를 통해 입체적인 장애물 인식과 객체 구분이 가능하다.

**SDK 설치**: `unitree_lidar_sdk`는 `build_first.sh` 실행 시 GitHub에서 자동으로 클론되어 시스템 라이브러리로 설치된다.

```bash
git clone https://github.com/UnitreeRobotics/unitree_lidar_sdk.git
cd unitree_lidar_sdk && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

센서가 연결되면 ROS2 드라이버가 `/unilidar/cloud` 토픽으로 `sensor_msgs/PointCloud2` 메시지를 퍼블리시한다. 이 토픽이 모든 탐지 파이프라인의 입력이 된다.

---

### 3.2 기본 탐지 노드: `ugv_lidar_detection`

#### 설계 목적

4D LiDAR 탐지의 첫 번째 구현으로, DBSCAN 클러스터링 기반의 단일 프레임 객체 탐지 노드를 작성했다. 시계열 트래킹 없이 매 프레임 독립적으로 객체를 탐지하기 때문에 구조가 단순하고, 탐지 알고리즘 자체를 검증하는 데 적합하다.

패키지 이름부터 `ugv_lidar_detection`으로 명명했고, `package.xml`의 description도 **"4D LiDAR Object Detection package"** 로 명시했다.

#### 탐지 파이프라인

```
[입력] /unilidar/cloud (PointCloud2)
         │
         ▼
[1단계] 높이 필터링 (Z축 임계값)
         - z_min = -0.3m  (지면 포인트 제거)
         - z_max =  2.5m  (천장/높은 구조물 제거)
         │
         ▼
[2단계] 다운샘플링
         - downsample_rate = 2  (N번째 포인트만 사용)
         - DBSCAN 연산 시간 단축 목적
         │
         ▼
[3단계] DBSCAN 클러스터링
         - eps = 0.6m        (같은 클러스터로 묶을 최대 거리)
         - min_samples = 10  (클러스터 최소 포인트 수)
         - 노이즈(-1) 제외
         │
         ▼
[4단계] 바운딩박스 추출
         - 각 클러스터의 min/max 좌표로 AABB 생성
         - 중심점 계산 (pose.position)
         │
         ▼
[출력] /lidar/detected_objects (MarkerArray, RViz 시각화)
```

**파라미터 런치 설정** (`launch/lidar_detection.launch.py`):

```python
parameters=[
    {'pointcloud_topic': '/unilidar/cloud'},
    {'z_min_filter': -0.3},
    {'z_max_filter':  2.5},
    {'cluster_eps':   0.6},
    {'cluster_min_samples': 10},
    {'downsample_rate': 2}
]
```

파라미터를 Launch 파일에서 주입하는 방식을 선택한 이유는, 실험 환경에 따라 지면 높이나 클러스터링 민감도를 소스코드 수정 없이 조정할 수 있게 하기 위해서다.

---

### 3.3 시계열 트래킹 노드: `pcd_cluster_pkg`

#### 설계 목적

기본 탐지 노드는 매 프레임 독립적으로 동작하기 때문에 동일한 물체가 프레임마다 다른 ID로 탐지되는 문제가 있다. 특히 **이동하는 물체를 정적인 물체와 구분**하고 해당 물체를 지속적으로 추적하려면 시간 축 정보가 필요하다.

이를 위해 `pcd_cluster_pkg`에서는 포인트 클라우드를 DBSCAN으로 클러스터링한 뒤, 프레임 간 클러스터 위치를 비교하여 동일 물체를 지속적으로 추적하는 **시계열 트래킹(Temporal Tracking)** 을 구현했다.

또한 odom 좌표계로 변환하여 로봇이 이동하더라도 물체의 절대 위치 변화를 정확히 측정할 수 있게 했다.

#### 7단계 파이프라인

**[1단계] ROI 필터링**

전체 포인트 클라우드에서 관심 영역(Region of Interest)만 추출한다. 너무 가까운 포인트(센서 노이즈)와 너무 먼 포인트(불필요한 배경)를 제거하고, 지면 포인트도 높이 임계값으로 제거한다.

```python
r = math.sqrt(x*x + y*y)
if self.roi_min_range < r < self.roi_max_range and z > self.ground_z_limit:
    # 0.3m ~ 15.0m 범위, 지면(z > 0.25m) 이상의 포인트만 유지
```

**[2단계] 로봇 자체 포인트 제거**

4D LiDAR가 로봇 차체에 장착되어 있기 때문에, 로봇 자신의 프레임이 포인트 클라우드에 나타난다. 이를 제거하지 않으면 로봇 자체가 객체로 탐지된다.

```python
self_box = x ∈ (-0.6, 0.6), y ∈ (-0.6, 0.6), z ∈ (-0.3, 0.8)
# 이 박스 내부 포인트를 모두 제거
```

**[3단계] 선택적 다운샘플링**

포인트 수가 3,000개를 초과하면 1/2 스트라이드로 다운샘플링하여 DBSCAN 처리 시간을 줄인다. 단순 스트라이드 방식이지만 공간 분포는 유지된다.

**[4단계] DBSCAN 클러스터링**

```python
clustering = DBSCAN(eps=0.7, min_samples=8).fit(points)
```

eps를 0.7m로 설정한 이유는 사람이나 소형 장애물의 포인트 밀도를 고려한 값이다. eps가 너무 작으면 하나의 물체가 여러 클러스터로 분리되고, 너무 크면 인접한 두 물체가 하나로 합쳐진다.

**[5단계] 추적 후보 필터링**

크기 제약 조건으로 실제 추적 대상이 될 수 있는 물체만 선별한다.

```python
is_trackable = (
    sx < 2.0m and sy < 2.0m and sz < 1.5m  # 크기 상한
    and cz > 0.2m                            # 지면 위 물체만
)
```

조건을 통과한 클러스터에 대해 TF 변환을 수행하여 센서 프레임 좌표를 odom 프레임 좌표로 변환한다. 이 odom 좌표가 프레임 간 비교의 기준이 된다.

**[6단계] 다중 프레임 트래킹 (Track Association)**

이전 프레임의 트랙 목록과 현재 탐지 목록을 거리 기반으로 매칭한다.

```python
# 거리가 가까운 쌍부터 우선 매칭 (Greedy 방식)
candidate_pairs.sort(key=lambda x: x[0])   # 거리 오름차순 정렬

# 매칭된 트랙의 운동량 누적
if move_threshold(0.05m) < motion_dist < max_motion_dist(3.0m):
    track['moving_count'] += 1

# is_moving_confirmed: moving_count >= min_motion_frames(1)
```

트랙이 매 프레임 탐지되지 않더라도 최대 12프레임(max_missed_frames)까지 유지한다. 일시적인 가림(occlusion)이 발생해도 트랙이 사라지지 않는다.

**[7단계] 타깃 선정 및 추적 제어**

이동 확정된 물체 중에서 추적 대상(lock target)을 선정하고, 해당 물체를 향해 로봇이 회전하도록 `/cmd_vel`을 퍼블리시한다.

```python
# 타깃 선정 기준: 정면에 가깝고, 가까운 물체 우선
score = angle + 0.15 * distance

# 제어 명령 생성
target_angle = math.atan2(cy, cx)
cmd.angular.z = angular_gain(1.5) * target_angle  # ±1.2 rad/s 클램핑
```

**잠금 안정성 (Lock Stability)** 은 특히 신경 쓴 부분이다. 새로운 물체가 나타날 때마다 타깃이 바뀌면 로봇이 불안정하게 진동한다. 이를 방지하기 위해 두 가지 안전장치를 추가했다.

- **lock_switch_cooldown (20프레임)**: 타깃을 변경하고 나서 20프레임 동안은 근처의 물체를 우선 선택하여 타깃이 쉽게 전환되지 않게 한다
- **rotation_freeze_threshold (0.20 rad/s)**: 로봇이 회전 중일 때는 새로운 타깃 선정을 중단하여 빠른 회전 중의 오탐지를 방지한다

---

### 3.4 운동 상태 시각화

추적 중인 물체의 상태를 RViz에서 색상으로 구분하여 직관적으로 확인할 수 있게 했다.

| 색상 | 상태 | 조건 |
|------|------|------|
| **초록 (Green)** | 정적 물체 | 운동량 미달 |
| **주황 (Orange)** | 운동 후보 | 직전 프레임 이동 감지 (0.05m 이상) |
| **빨강 (Red)** | 운동 확정 | 누적 이동 프레임 ≥ 1회 |
| **파랑 (Blue)** | 잠금 타깃 | 현재 추적 중인 대상 |

바운딩박스는 `Marker.LINE_LIST` 타입으로 8개 꼭짓점과 12개 모서리를 직접 계산하여 그린다. 이 방식이 `Marker.CUBE`보다 내부가 투명하게 표시되어 포인트 클라우드와 겹쳐보기에 적합하다.

---

### 3.5 지면 분리: `plane_fit_pkg`

#### 설계 목적

포인트 클라우드에서 지면 포인트를 정확히 분리해야 실제 장애물 포인트만 탐지 알고리즘에 전달할 수 있다. 단순한 Z축 임계값 방식은 경사로나 요철 지형에서 부정확하다. 이를 개선하기 위해 **RANSAC 기반 평면 피팅**과 **Kalman 필터**를 결합한 지면 분리 노드를 구현했다.

#### 3가지 알고리즘 비교 구현

실험 목적으로 3가지 알고리즘을 동시에 실행하고 성능을 비교한다. 매 프레임 처리 시간(ms), Inlier 비율, RMSE, 각도 오차가 로그로 출력된다.

**① Open3D RANSAC** (레퍼런스 구현)

```python
pcd.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=500)
```

`open3d` 라이브러리의 구현을 레퍼런스로 사용했다. C++ 내부 구현으로 빠르지만 `open3d` 의존성이 필요하다.

**② Custom RANSAC** (순수 NumPy 구현)

```python
for _ in range(500):
    ids = random.sample(range(n_points), 3)
    plane = fit_plane_from_3points(p1, p2, p3)
    inliers = where(point_plane_distances(points, plane) < 0.05)
    if len(inliers) > len(best_inliers):
        best_plane, best_inliers = plane, inliers
```

외부 라이브러리 없이 NumPy만으로 동일한 알고리즘을 구현하여, `open3d`를 설치하기 어려운 환경에서도 동작하는 폴백(fallback)으로 사용한다.

**③ RANSAC + LS + Kalman** (최종 채택)

Open3D RANSAC으로 초기 inlier를 구한 뒤, 해당 inlier 포인트들로 **최소자승법(SVD)**을 사용하여 평면 계수를 정밀 보정한다. 여기에 **Kalman 필터**를 적용하여 프레임 간 평면 계수의 시간적 안정성을 확보한다.

```python
# 1) Open3D RANSAC → 초기 inlier 획득
# 2) inlier 포인트로 SVD 기반 최소자승 평면 보정
_, _, vh = np.linalg.svd(centered_points, full_matrices=False)
normal = vh[-1, :]  # 가장 작은 특이값에 해당하는 법선벡터

# 3) Kalman 필터로 평면 계수 [a, b, c, d] 스무딩
plane_ls = self.kalman.update(plane_ls_raw)
```

**Kalman 필터 설정**:
- 상태 벡터: 평면 계수 `[a, b, c, d]` (4차원)
- 프로세스 노이즈 Q = I × 0.001 (평면이 급격히 바뀌지 않는다는 가정)
- 측정 노이즈 R = I × 0.01

결과적으로 RANSAC 단독 대비 RMSE가 낮아지고, 프레임 간 평면 계수의 진동이 줄어 안정적인 지면 분리가 가능해졌다.

**지원 검출 모드**:

```python
'ground' → 기준 법선: [0, 0, 1]        (수평 지면)
'ramp'   → 기준 법선: [sin15°, 0, cos15°]  (15° 경사로)
'wall_x' → 기준 법선: [1, 0, 0]        (X축 방향 벽)
'wall_y' → 기준 법선: [0, 1, 0]        (Y축 방향 벽)
```

**RViz 시각화**: Inlier(지면으로 분류된 포인트)는 녹색, outlier(장애물로 분류된 포인트)는 빨간색으로 표시된다. 또한 화면 상단에 3가지 알고리즘의 성능 지표를 텍스트 마커로 오버레이한다.

---

### 3.6 3D→2D 변환: `pcd_to_scan_pkg`

4D LiDAR의 3D 포인트 클라우드를 2D `LaserScan`으로 변환하는 노드다. Nav2의 일부 플래너나 SLAM 알고리즘은 2D 레이저 스캔만 입력으로 받기 때문에, 3D 포인트 클라우드를 2D로 투영하는 변환 레이어가 필요하다.

특정 높이 범위의 포인트만 선택하여 2D 평면에 투영하는 방식으로, Livox 센서와도 호환 가능한 범용 변환 노드로 설계했다.

---

## 4. 시스템 통합 구조

### 4.1 ROS2 토픽 흐름도

```
[Unitree 4D LiDAR 드라이버]
          │
          │ /unilidar/cloud (PointCloud2)
          ▼
┌─────────────────────┐     ┌─────────────────────┐
│ lidar_detector_node  │     │   plane_fit_node      │
│ (ugv_lidar_detection)│     │   (plane_fit_pkg)      │
│                      │     │                       │
│ DBSCAN 단일프레임    │     │ RANSAC + LS + Kalman  │
└──────────┬───────────┘     └──────────┬────────────┘
           │                            │
           │ /lidar/detected_objects     │ /plane_fit/markers
           │ (MarkerArray)              │ (MarkerArray)
           ▼                            ▼
      [RViz 시각화]              [RViz 시각화]


[포인트 클라우드 입력]
          │
          │ /mid360_PointCloud2 (or /unilidar/cloud)
          ▼
┌─────────────────────────────────────────┐
│           cluster_node                   │
│           (pcd_cluster_pkg)              │
│                                          │
│  ROI 필터 → 자체포인트 제거 → DBSCAN    │
│  → 트랙 연관 → 운동 분류 → 타깃 선정   │
└────────────┬────────────────────────────┘
             │ /cluster_markers (MarkerArray)
             │ /cmd_vel (Twist)
             ▼
       [RViz + 로봇 제어]
```

### 4.2 패키지 구성 요약

| 패키지 | 역할 | 입력 토픽 | 출력 토픽 |
|--------|------|-----------|-----------|
| `ugv_lidar_detection` | 단일 프레임 4D LiDAR 탐지 | `/unilidar/cloud` | `/lidar/detected_objects` |
| `pcd_cluster_pkg` | 시계열 트래킹 + 타깃 추적 제어 | `/mid360_PointCloud2`, `/odom` | `/cluster_markers`, `/cmd_vel` |
| `plane_fit_pkg` | RANSAC 지면 분리 + Kalman 안정화 | `/mid360_PointCloud2` | `/plane_fit/markers` |
| `pcd_to_scan_pkg` | 3D→2D LaserScan 변환 | PointCloud2 | `LaserScan` |

### 4.3 주요 파라미터 요약

| 항목 | 값 | 위치 |
|------|----|------|
| LiDAR 입력 토픽 | `/unilidar/cloud` | lidar_detection.launch.py |
| ROI 범위 | 0.3m ~ 15.0m | cluster_node.py |
| 지면 높이 임계 | z > 0.25m | cluster_node.py |
| DBSCAN eps (단순) | 0.6m | lidar_detection.launch.py |
| DBSCAN eps (트래킹) | 0.7m | cluster_node.py |
| 운동 감지 임계 | 0.05m/프레임 | cluster_node.py |
| 트랙 유지 프레임 | 12프레임 | cluster_node.py |
| 타깃 전환 쿨다운 | 20프레임 | cluster_node.py |
| RANSAC 반복 횟수 | 500회 | plane_fit_node.py |
| Kalman Q | I × 0.001 | plane_fit_node.py |
| Kalman R | I × 0.01 | plane_fit_node.py |

---

*본 연구일지는 FSD_Vehicle 프로젝트의 개발 환경 구성 및 Unitree 4D LiDAR 탐지 기술 구현 내용을 기록한 문서입니다.*
