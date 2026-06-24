#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"
waver_field_env_require

echo "[WAVER_BACKEND] starting field backend through Jetson Docker"
echo "[WAVER_BACKEND] target=${JETSON_USER}@${JETSON_HOST}:${JETSON_PORT:-22} ws=${JETSON_WS} container=${CONTAINER}"
exec bash "${ROOT}/scripts/waver_field_docker_backend_start.sh"
