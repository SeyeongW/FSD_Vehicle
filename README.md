# FSD Vehicle - UGV ROS2 개발 환경

**Full Self-Driving Vehicle** 프로젝트 - Livox Mid-360 3D 라이다 탑재 UGV 자율주행 개발 환경

## 개발 스택

| 항목 | 내용 |
|------|------|
| OS | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| 시뮬레이터 | Gazebo |
| 라이다 | Livox Mid-360 |
| 카메라 | Intel RealSense D435i |
| 실행 플랫폼 | x86_64 PC / Jetson Orin Nano Super (JetPack 6) |

---

## 빠른 시작 (새 팀원 - 모든 PC 동일)

### 1. 저장소 클론

```bash
git clone https://github.com/SeyeongW/FSD_Vehicle.git
cd FSD_Vehicle
```

### 2. 로봇 모델 설정

`docker/.env` 파일에서 원하는 모델로 변경:
```bash
UGV_MODEL=ugv_rover   # ugv_rover / ugv_beast / rasp_rover
```

### 3. Docker 이미지 빌드 & 실행

```bash
# PC 개발환경 (첫 빌드 10~20분 소요)
# Windows (cmd/powershell):
build_pc
run_pc

# Linux / WSL:
make build_pc
make run_pc
# (또는 ./build_pc.sh / ./run_pc.sh)
```

### 4. 컨테이너 안에서 ROS 빌드 (최초 1회)

```bash
bash build_first.sh    # 외부 패키지 포함 전체 빌드
```

### 5. 이후 개발 시

```bash
# 컨테이너 실행
bash docker/run.sh pc

# 코드 수정 후 빌드
bash build_common.sh
```

---

## Jetson Orin Nano Super 배포

```bash
# Jetson에서 클론
git clone https://github.com/SeyeongW/FSD_Vehicle.git
cd FSD_Vehicle

# Jetson 이미지 빌드 & 실행 (30~40분 소요)
bash docker/run.sh build-jetson
bash docker/run.sh jetson

# 컨테이너 안에서 빌드
bash build_first.sh
```

---

## 자주 쓰는 명령어

| 명령어 | 설명 |
|--------|------|
| `run_pc` | PC 개발환경 시작 |
| `bash docker/run.sh jetson` | Jetson 배포환경 시작 |
| `bash docker/run.sh stop` | 컨테이너 중지 |
| `build_pc` | PC 이미지 빌드 |
| `bash docker/run.sh build-jetson` | Jetson 이미지 빌드 |
| `bash build_common.sh` | ugv_main 패키지만 빌드 |
| `bash build_first.sh` | 전체 빌드 (최초 1회) |

---

## 브랜치 전략

```
main          ← 안정 버전 (배포용)
develop       ← 개발 통합 브랜치
feature/xxx   ← 기능 개발 브랜치
```

---

## 패키지 구조

```
FSD_Vehicle/
├── docker/
│   ├── Dockerfile          ← PC 개발환경 이미지
│   ├── Dockerfile.jetson   ← Jetson 배포 이미지
│   ├── entrypoint.sh       ← 컨테이너 시작 스크립트
│   ├── .env                ← 환경변수 (UGV_MODEL 등)
│   └── run.sh              ← 편의 실행 스크립트
├── docker-compose.yml      ← PC 개발용
├── docker-compose.jetson.yml ← Jetson 배포용
└── src/
    ├── ugv_main/           ← 메인 패키지 (직접 개발)
    │   ├── ugv_description/    ← URDF, 메쉬 (Livox Mid-360 포함)
    │   ├── ugv_gazebo/         ← Gazebo 시뮬레이션
    │   ├── ugv_bringup/        ← 실제 로봇 bringup
    │   ├── ugv_nav/            ← 자율주행 네비게이션
    │   └── ugv_slam/           ← SLAM (LiDAR 기반)
    ├── ugv_else/           ← 외부 패키지 (cartographer, teb 등)
    ├── Livox-SDK2/             ← Livox SDK
    ├── livox_ros_driver2/      ← Livox ROS2 드라이버
    └── livox_laser_simulation_RO2/  ← Livox Gazebo 시뮬레이션
```

---

## 라이선스

이 프로젝트는 Apache-2.0 라이선스를 따릅니다.