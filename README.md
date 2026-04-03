# UGV ROS2 개발 환경

Livox Mid-360 3D 라이다가 탑재된 UGV(Unmanned Ground Vehicle) ROS2 개발 환경입니다.

## 환경 요구사항

- Docker + Docker Compose
- NVIDIA GPU (선택사항, RViz 가속)
- X11 (GUI 사용 시)

## 빠른 시작 (새 팀원 - 모든 PC 동일)

```bash
# 1. 저장소 클론
git clone https://github.com/<팀_계정>/ugv_ws.git
cd ugv_ws

# 2. Docker 이미지 빌드 (최초 1회 - 10~20분)
bash docker/run.sh build-pc

# 3. 개발환경 시작
bash docker/run.sh pc

# 4. 컨테이너 안에서 최초 빌드 (최초 1회)
bash build_first.sh

# 5. 이후 빌드 (코드 수정 후)
bash build_common.sh
```

## 로봇 모델 설정

`docker/.env` 파일에서 변경:
```bash
UGV_MODEL=ugv_rover   # ugv_rover / ugv_beast / rasp_rover
```

## 자주 쓰는 명령어

| 명령어 | 설명 |
|--------|------|
| `bash docker/run.sh pc` | PC 개발환경 시작 |
| `bash docker/run.sh jetson` | Jetson 환경 시작 |
| `bash docker/run.sh stop` | 컨테이너 중지 |
| `bash build_common.sh` | ugv_main 패키지 빌드 |

## Jetson Orin Nano Super 배포

```bash
# Jetson에서
git clone https://github.com/<팀_계정>/ugv_ws.git
cd ugv_ws
bash docker/run.sh build-jetson
bash docker/run.sh jetson
```

## 브랜치 전략

```
main          ← 안정 버전 (배포용)
develop       ← 개발 통합 브랜치
feature/xxx   ← 기능 개발 브랜치
```

## 센서 구성

- Livox Mid-360 3D LiDAR
- Intel RealSense D435i (3D 카메라)
- IMU

## 패키지 구조

```
src/
├── ugv_main/         ← 메인 패키지 (직접 개발)
│   ├── ugv_description/  ← URDF, 메쉬
│   ├── ugv_gazebo/       ← Gazebo 시뮬레이션
│   ├── ugv_bringup/      ← 실제 로봇 실행
│   ├── ugv_nav/          ← 자율주행 네비게이션
│   └── ugv_slam/         ← SLAM
├── ugv_else/         ← 외부 패키지 (수정 없이 사용)
├── livox_ros_driver2/    ← Livox 드라이버
└── livox_laser_simulation_RO2/ ← Livox 시뮬레이션
```