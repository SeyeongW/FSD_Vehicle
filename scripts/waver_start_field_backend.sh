#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"
waver_field_env_require

WAVER_REAL_PROFILE="${WAVER_REAL_PROFILE:-bird_patrol_production}"
FIELD_READINESS_LEVEL="${FIELD_READINESS_LEVEL:-L5}"
FIELD_READINESS_STRICT="${FIELD_READINESS_STRICT:-true}"
export WAVER_REAL_PROFILE FIELD_READINESS_LEVEL FIELD_READINESS_STRICT

echo "[WAVER_BACKEND] starting strict field backend through Jetson Docker"
echo "[WAVER_BACKEND] target=${JETSON_USER}@${JETSON_HOST}:${JETSON_PORT:-22} ws=${JETSON_WS} container=${CONTAINER}"
echo "[WAVER_BACKEND] profile=${WAVER_REAL_PROFILE} readiness=${FIELD_READINESS_LEVEL} strict=${FIELD_READINESS_STRICT}"
if bash "${ROOT}/scripts/waver_field_lidar_nav_backend_start.sh"; then
  echo "FIELD_BACKEND_READY=PASS"
else
  rc=$?
  echo "FIELD_BACKEND_READY=FAIL"
  exit "${rc}"
fi
