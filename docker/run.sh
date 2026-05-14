#!/bin/bash
# =============================================================
# FSD_Vehicle Docker 빠른 실행 스크립트 (Jetson 전용)
# 사용법: bash docker/run.sh [jetson|stop|build-jetson]
# =============================================================

TARGET=${1:-jetson}

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
  else
    echo -e "\033[32m[OK] 장치 확인 완료: $port\033[0m"
  fi
}

case "$TARGET" in
  jetson)
    echo "[FSD_Vehicle] Jetson 환경 시작..."
    check_devices "$SERIAL_PORT_JETSON"
    docker compose -f docker-compose.jetson.yml up -d --build
    docker compose -f docker-compose.jetson.yml exec fsd-jetson bash
    ;;
  stop)
    echo "[FSD_Vehicle] 컨테이너 중지..."
    docker compose -f docker-compose.jetson.yml down 2>/dev/null || true
    ;;
  build-jetson)
    echo "[FSD_Vehicle] Jetson 이미지 빌드..."
    docker compose -f docker-compose.jetson.yml build
    ;;
  *)
    echo "사용법: bash docker/run.sh [jetson|stop|build-jetson]"
    echo ""
    echo "  jetson       Jetson 환경 시작 (CUDA, YOLO 포함)"
    echo "  stop         컨테이너 중지"
    echo "  build-jetson Jetson 이미지 빌드"
    ;;
esac
