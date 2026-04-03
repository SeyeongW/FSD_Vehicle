#!/bin/bash
# =============================================================
# UGV Docker 빠른 실행 스크립트
# 사용법: bash docker/run.sh [pc|jetson] [명령어]
# =============================================================

TARGET=${1:-pc}

# X11 접근 허용 (GUI 사용 시)
xhost +local:docker 2>/dev/null || true

case "$TARGET" in
  pc)
    echo "[UGV] PC 개발환경 시작..."
    docker compose up -d --build
    docker compose exec ugv-dev bash
    ;;
  jetson)
    echo "[UGV] Jetson 환경 시작..."
    docker compose -f docker-compose.jetson.yml up -d --build
    docker compose -f docker-compose.jetson.yml exec ugv-jetson bash
    ;;
  stop)
    echo "[UGV] 컨테이너 중지..."
    docker compose down
    docker compose -f docker-compose.jetson.yml down 2>/dev/null || true
    ;;
  build-pc)
    echo "[UGV] PC 이미지 빌드..."
    docker compose build
    ;;
  build-jetson)
    echo "[UGV] Jetson 이미지 빌드..."
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
