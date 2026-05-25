#!/bin/bash
# =============================================================
# FSD_Vehicle 로컬 환경 설정 스크립트
# 사용법: source setup_local_env.sh
#
# 이 스크립트는 워크스페이스 경로를 자동 감지하여
# UGV_WS_PATH 환경변수를 설정합니다.
# Docker가 아닌 로컬에서 직접 실행할 때 사용하세요.
# =============================================================

# 스크립트 위치 기반으로 워크스페이스 경로 자동 감지
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# .env 파일에서 환경변수 로드 (UGV_WS_PATH 제외)
if [ -f "${SCRIPT_DIR}/.env" ]; then
    while IFS='=' read -r key value; do
        # 주석과 빈 줄 무시, UGV_WS_PATH는 아래에서 자동 설정
        [[ "$key" =~ ^#.*$ || -z "$key" || "$key" == "UGV_WS_PATH" ]] && continue
        # 값에서 앞뒤 공백 제거
        value="${value%%\#*}"    # 인라인 주석 제거
        value="${value%"${value##*[! ]}"}"  # 뒤 공백 제거
        export "$key"="$value"
    done < "${SCRIPT_DIR}/.env"
fi

# UGV_WS_PATH를 항상 현재 스크립트 위치 기준으로 자동 설정
export UGV_WS_PATH="${SCRIPT_DIR}"

echo "============================================="
echo " FSD_Vehicle 로컬 환경 설정 완료"
echo " UGV_WS_PATH = ${UGV_WS_PATH}"
echo " UGV_MODEL   = ${UGV_MODEL:-not set}"
echo " ROS_DOMAIN_ID = ${ROS_DOMAIN_ID:-0}"
echo "============================================="
