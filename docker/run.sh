#!/bin/bash
# =============================================================
# FSD_Vehicle Docker 빠른 실행 스크립트
# 사용법: bash docker/run.sh [pc|jetson] [명령어]
# =============================================================

TARGET=${1:-pc}

# X11 접근 허용 (GUI 사용 시)
xhost +local:docker 2>/dev/null || true

# 환경변수 로드 (프로젝트 루트의 .env 우선)
if [ -f ".env" ]; then
  source .env
elif [ -f "docker/.env" ]; then
  source docker/.env
fi

# 장치 확인 함수
check_devices() {
  local port=$1
  if [ -z "$port" ]; then return; fi
  if [ ! -e "$port" ]; then
    echo -e "\033[33m[WARNING] 장치를 찾을 수 없습니다: $port\033[0m"
    echo "실제 로봇이 아니거나 연결되지 않았을 수 있습니다. 시뮬레이션은 계속 가능합니다."
  else
    echo -e "\033[32m[OK] 장치 확인 완료: $port\033[0m"
  fi
}

case "$TARGET" in
  pc)
    echo "[FSD_Vehicle] PC 개발환경 시작..."
    check_devices "$SERIAL_PORT_PC"
    docker compose up -d --build
    docker compose exec fsd-dev bash
    ;;
  jetson)
    echo "[FSD_Vehicle] Jetson 환경 시작..."
    check_devices "$SERIAL_PORT_JETSON"
    docker compose -f docker-compose.jetson.yml up -d --build
    docker compose -f docker-compose.jetson.yml exec fsd-jetson bash
    ;;
  stop)
    echo "[FSD_Vehicle] 컨테이너 중지..."
    docker compose down
    docker compose -f docker-compose.jetson.yml down 2>/dev/null || true
    ;;
  build-pc)
    echo "[FSD_Vehicle] PC 이미지 빌드..."
    docker compose build
    ;;
  build-jetson)
    echo "[FSD_Vehicle] Jetson 이미지 빌드..."
    docker compose -f docker-compose.jetson.yml build
    ;;
  *)
    echo "사용법: bash docker/run.sh [pc|jetson|stop|build-pc|build-jetson]"
    echo ""
    echo "  pc           PC 개발환경 시작 (RViz, Gazebo 포함)"
    echo "  jetson       Jetson 환경 시작 (CUDA, YOLO 포함)"
    echo "  stop         모든 컨테이너 중지"
    echo "  build-pc     PC 이미지만 빌드"
    echo "  build-jetson Jetson 이미지만 빌드"
    ;;
esac
