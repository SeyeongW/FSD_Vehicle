# 🏎️ FSD_Vehicle PRO Project
**FSD (Formula Student Driverless) 기반 UGV 자율주행 연구를 위한 ROS2 기반 통합 개발 환경**

이 저장소는 **PC(시뮬레이션/개발)** 및 **Jetson Orin Nano(실제 주행)** 환경을 모두 지원하는 Docker 기반 워크스페이스입니다.

---

## 🚀 퀵 스타트 가이드 (Quick Start)

처음 환경을 구축하는 팀원들은 다음 순서대로 진행하세요.

### 1. 전제 조건 (Prerequisites)
- **Linux PC**: 추천 (Ubuntu 22.04 LTS)
- **Docker**: [설치 가이드](https://docs.docker.com/engine/install/ubuntu/)
- **Docker Compose**: [설치 가이드](https://docs.docker.com/compose/install/)

### 2. 저장소 클론 (Clone Repository)
원하는 작업 폴더(예: `~/ros2_ws`)를 만들고 그 안에서 클론을 진행합니다.
```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/SeyeongW/FSD_Vehicle.git ugv_ws
cd ugv_ws
```

### 3. 개발 이미지 빌드 (Build Image)
본인의 장치에 맞는 이미지를 빌드합니다. (약 5~10분 소요)
```bash
# PC에서 개발/시뮬레이션 시
bash docker/run.sh build-pc

# 실제 Jetson Orin Nano에서 주행 시
bash docker/run.sh build-jetson
```

### 4. 컨테이너 실행 (Run Container)
```bash
# PC 개발 시
bash docker/run.sh pc

# Jetson 주행 시
bash docker/run.sh jetson
```
> [!TIP]
> 실제 로봇이 연결되지 않았을 경우 장치 확인 경고가 뜰 수 있으나, **무시하고 엔터**를 누르면 시뮬레이션 모드로 정상 실행됩니다.

### 5. 초기 전체 빌드 (Initial Build)
컨테이너 안으로 접속되면, 최초 1회 전체 빌드가 필요합니다.
```bash
# 1. 아프릴태그 라이브러리 빌드 (필수)
bash build_apriltag.sh

# 2. 전체 ROS2 패키지 빌드
bash build_first.sh
```

---

## 🛠️ 개발 워크플로우 (Development Workflow)

코드를 수정했거나 새로운 패키지를 추가한 경우, 컨테이너 내부에서 다음 명령어를 사용하세요.

```bash
# 코드 수정 후 점진적 빌드
bash build_common.sh

# 환경 변수 리로드 (빌드 후 필수)
source install/setup.bash
```

---

## 🖥️ 시뮬레이션 및 실제 로봇 실행 (Execution)

### 🌍 Gazebo 시뮬레이션 (PC 전용)
하드웨어 연결 없이 가상의 환경에서 로봇을 구동합니다.
```bash
# 컨테이너 내부에서 실행
ros2 launch ugv_gazebo slam_nav.launch.py
```

### 🤖 실제 로봇 주행 (Hardware)
Jetson에서 실제 센서와 모터를 구동합니다.
```bash
# 컨테이너 내부에서 실행
ros2 launch ugv_bringup bringup_lidar.launch.py
```

---

## 📁 프로젝트 구조 (Architecture)

- `src/ugv_main`: 우리가 직접 개발하는 핵심 제어/인식 소스코드
- `src/ugv_else`: 3rd party 라이브러리 및 센서 드라이버 (Livox, LDLiDAR 등)
- `docker/`: Dockerfile 및 환경 설정 파일
- `.env`: 독커 이미지 이름, 컨테이너 이름, 시리얼 포트 등 중요 설정 제어

---

## ⚠️ 주의사항 (Notes)
- **GUI 지원**: PC에서 실행 시 RViz와 Gazebo를 띄우기 위해 호스트의 X11 접근 권한을 자동으로 허용(`xhost +`)합니다.
- **볼륨 마운트**: 호스트의 `src` 폴더와 독커 내부가 링크되어 있습니다. 호스트에서 코드를 수정하면 독커 내부에 즉시 반영됩니다.
- **하드웨어 권한**: 실제 로봇 연결 시 시리얼 포트 접근 권한(`dialout` 그룹)이 자동으로 포함되어 있습니다.

---

**관리자: SeyeongW**  
문제 발생 시 Issue를 남기거나 관리자에게 문의하세요.