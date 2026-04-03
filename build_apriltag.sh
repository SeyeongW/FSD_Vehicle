#!/bin/bash
# apriltag C 라이브러리 빌드 스크립트
# 호스트/Docker 컨테이너 양쪽에서 동작

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APRILTAG_DIR="$WS_ROOT/src/ugv_else/apriltag_ros/apriltag"

echo "[apriltag] 워크스페이스 루트: $WS_ROOT"

# 1. apriltag 디렉토리로 이동
cd "$APRILTAG_DIR" || { echo "[ERROR] 경로 없음: $APRILTAG_DIR"; exit 1; }

# 2. 기존 빌드 캐시 제거 (경로 불일치 방지)
if [ -d "build" ]; then
    echo "[apriltag] 기존 build 디렉토리 제거 중..."
    rm -rf build
fi

# 3. 빌드 설정
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 4. 설치 (root면 sudo 불필요)
if [ "$(id -u)" -eq 0 ]; then
    cmake --build build --target install
else
    sudo cmake --build build --target install
fi

# 5. 워크스페이스 루트로 복귀
cd "$WS_ROOT"
echo "[apriltag] 빌드 완료!"