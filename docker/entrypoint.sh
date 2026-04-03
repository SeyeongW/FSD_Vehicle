#!/bin/bash
# =============================================================
# UGV Docker Entrypoint
# =============================================================
set -e

# ── ROS2 환경 소싱 ───────────────────────────────────────
source /opt/ros/humble/setup.bash

# ── 워크스페이스 소싱 (빌드되어 있으면) ─────────────────
if [ -f /ros2_ws/ugv_ws/install/setup.bash ]; then
    source /ros2_ws/ugv_ws/install/setup.bash
    echo "[UGV] Workspace sourced: /ros2_ws/ugv_ws/install/setup.bash"
else
    echo "[UGV] Workspace not built yet. Run build scripts inside container."
fi

# ── 환경변수 기본값 설정 ─────────────────────────────────
export UGV_MODEL=${UGV_MODEL:-ugv_rover}
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo ""
echo "========================================"
echo "  UGV ROS2 Development Environment"
echo "  Model  : $UGV_MODEL"
echo "  Domain : $ROS_DOMAIN_ID"
echo "========================================"
echo ""
echo "Build scripts:"
echo "  bash build_first.sh    ← 최초 빌드 (외부 패키지 포함)"
echo "  bash build_common.sh   ← ugv_main 패키지 빌드"
echo ""

exec "$@"
