#!/usr/bin/env bash
set -euo pipefail

JETSON_HOST_ARG="${1:-${JETSON_HOST:-10.139.225.150}}"
JETSON_USER_ARG="${2:-${JETSON_USER:-sw}}"
JETSON_WS_ARG="${3:-${JETSON_WS:-/home/sw/ros2_ws5/FSD_Vehicle}}"
OUT="${WAVER_HOME_FIELD_ENV:-$HOME/.waver_field_env}"

if [ ! -t 0 ]; then
  echo "[WAVER_ENV][ERROR] interactive terminal required to create ${OUT}" >&2
  exit 1
fi

printf '[WAVER_ENV] Jetson SSH password for %s@%s: ' "${JETSON_USER_ARG}" "${JETSON_HOST_ARG}" >&2
IFS= read -r -s JETSON_PASS_ARG
printf '\n' >&2

if [ -z "${JETSON_PASS_ARG}" ]; then
  echo "[WAVER_ENV][ERROR] empty password; not writing ${OUT}" >&2
  exit 1
fi

tmp="$(mktemp)"
{
  echo "# Waver field local secrets and host override."
  echo "# This file lives outside the repository. Do not share it publicly."
  echo "JETSON_HOST=${JETSON_HOST_ARG}"
  echo "JETSON_USER=${JETSON_USER_ARG}"
  echo "JETSON_PORT=22"
  echo "JETSON_WS=${JETSON_WS_ARG}"
  echo "CONTAINER=fsd_dev_jetson"
  echo "ROS_DOMAIN_ID=0"
  echo "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
  echo "WAVER_ALLOW_PASSWORD_SSH=1"
  echo "WAVER_PASSWORD_BOOTSTRAP=false"
  printf 'JETSON_PASS=%q\n' "${JETSON_PASS_ARG}"
} > "${tmp}"

install -m 600 "${tmp}" "${OUT}"
rm -f "${tmp}"

echo "[WAVER_ENV] wrote ${OUT} with mode 600"
echo "[WAVER_ENV] Next:"
echo "  cd ~/ros2_ws5/FSD_Vehicle"
echo "  WAVER_REAL_PROFILE=lidar_nav_backend FIELD_READINESS_LEVEL=L2 bash scripts/waver_start_field_backend.sh"
echo "  bash scripts/waver_field_local_ui_start.sh"
