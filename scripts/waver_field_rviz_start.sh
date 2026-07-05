#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

DRY_RUN=false
USE_OPERATOR_PANEL=false
REQUIRE_JETSON_CHECK="${WAVER_SKIP_JETSON_CHECK:-false}"
RVIZ_CONFIG="${WAVER_RVIZ_CONFIG:-}"
MAP_TOPIC="${WAVER_MAP_TOPIC:-/map}"
SCAN_TOPIC="${WAVER_SCAN_TOPIC:-/scan_safety}"
POINTCLOUD_TOPIC="${WAVER_POINTCLOUD_TOPIC:-/livox/lidar}"
FIXED_FRAME="${WAVER_FIXED_FRAME:-map}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dry-run) DRY_RUN=true; shift ;;
    --with-panel) USE_OPERATOR_PANEL=true; shift ;;
    --map-topic) MAP_TOPIC="$2"; shift 2 ;;
    --scan-topic) SCAN_TOPIC="$2"; shift 2 ;;
    --pointcloud-topic) POINTCLOUD_TOPIC="$2"; shift 2 ;;
    --fixed-frame) FIXED_FRAME="$2"; shift 2 ;;
    --rviz-config) RVIZ_CONFIG="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage: bash scripts/waver_field_rviz_start.sh [options]

Options:
  --dry-run                      Print the command without launching RViz.
  --with-panel                   Also launch the local waver_remote_panel.
  --map-topic TOPIC              Map topic label for operator context.
  --scan-topic TOPIC             Scan topic label for operator context.
  --pointcloud-topic TOPIC       PointCloud topic label for operator context.
  --fixed-frame FRAME            Expected RViz fixed frame, default map.
  --rviz-config PATH             RViz config override.

This script is local-PC only. Real robot backend nodes stay in Jetson Docker.
EOF
      exit 0
      ;;
    *) echo "[LOCAL_RVIZ][ERROR] unknown arg: $1" >&2; exit 2 ;;
  esac
done

# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"
waver_missing_env=()
for name in JETSON_HOST JETSON_USER JETSON_WS CONTAINER ROS_DOMAIN_ID SERIAL_PORT; do
  value="${!name:-}"
  if _waver_is_placeholder "${value}"; then
    waver_missing_env+=("${name}")
  fi
done
if [ "${DRY_RUN}" = "false" ]; then
  waver_field_env_require
  waver_field_env_ensure_password
fi

if [ "${DRY_RUN}" = "false" ] && [ "${REQUIRE_JETSON_CHECK}" != "true" ]; then
  waver_ssh_cmd
  echo "[LOCAL_RVIZ] checking Jetson SSH ${JETSON_USER}@${JETSON_HOST}:${JETSON_PORT:-22}"
  "${WAVER_SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "docker ps --format '{{.Names}}' | grep -qx '${CONTAINER}'" >/tmp/waver_field_rviz_jetson_check.log 2>&1 || {
    echo "[LOCAL_RVIZ][ERROR] Jetson Docker container is not reachable: ${CONTAINER}" >&2
    tail -20 /tmp/waver_field_rviz_jetson_check.log >&2 || true
    echo "[LOCAL_RVIZ][ERROR] Set WAVER_SKIP_JETSON_CHECK=true only for Gazebo/local replay." >&2
    exit 13
  }
fi

CMD=(ros2 launch waver_patrol remote_visualization.launch.py
  use_rviz:=true
  use_operator_panel:="${USE_OPERATOR_PANEL}"
  map_topic:="${MAP_TOPIC}"
  scan_topic:="${SCAN_TOPIC}"
  pointcloud_topic:="${POINTCLOUD_TOPIC}"
  fixed_frame:="${FIXED_FRAME}"
)
if [ -n "${RVIZ_CONFIG}" ]; then
  CMD+=(rviz_config:="${RVIZ_CONFIG}")
fi

echo "[LOCAL_RVIZ] fixed_frame=${FIXED_FRAME} map=${MAP_TOPIC} scan=${SCAN_TOPIC} pointcloud=${POINTCLOUD_TOPIC}"
if [ "${DRY_RUN}" = "true" ]; then
  if [ "${#waver_missing_env[@]}" -gt 0 ]; then
    echo "DRY_RUN_MISSING_ENV=${waver_missing_env[*]}"
  fi
  printf 'DRY_RUN_CMD='
  printf '%q ' "${CMD[@]}"
  printf '\n'
  exit 0
fi

set +u
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  set -u
  echo "[LOCAL_RVIZ][ERROR] install/setup.bash missing. Build local visualization packages first." >&2
  echo "[LOCAL_RVIZ][ERROR] Example: colcon build --symlink-install --packages-up-to ugv_tools waver_patrol" >&2
  exit 2
fi
set -u

exec "${CMD[@]}"
