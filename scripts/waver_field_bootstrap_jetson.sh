#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"
waver_field_env_require
waver_field_env_ensure_password
waver_ssh_cmd
waver_rsync_cmd

echo "[BOOTSTRAP] target=${JETSON_USER}@${JETSON_HOST}:${JETSON_PORT:-22} ws=${JETSON_WS} container=${CONTAINER}"

"${WAVER_SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "mkdir -p '$(dirname "${JETSON_WS}")'"

EXCLUDES=(
  --exclude .git/
  --exclude build/
  --exclude install/
  --exclude log/
  --exclude build_docker/
  --exclude install_docker/
  --exclude log_docker/
  --exclude __pycache__/
  --exclude '*.pyc'
  --exclude config/waver_field_env.local
  --exclude .env.local
  --exclude '*.env.local'
  --exclude '*.bag'
  --exclude '*.db3'
  --exclude '*.mcap'
)

echo "[BOOTSTRAP] rsync repository to Jetson"
"${WAVER_RSYNC_CMD[@]}" "${EXCLUDES[@]}" "${ROOT}/" "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/"

echo "[BOOTSTRAP] prepare Docker container and ROS overlay"
"${WAVER_SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "bash -s" -- "${JETSON_WS}" "${CONTAINER}" "${FIELD_BUILD_IN_DOCKER}" <<'REMOTE'
set -euo pipefail
JETSON_WS="$1"
CONTAINER="$2"
FIELD_BUILD_IN_DOCKER="$3"

cd "${JETSON_WS}"
test -f .env
test -f config/waver_field_env
docker compose -f docker-compose.jetson.yml config >/tmp/waver_compose_config.out
bash docker/run.sh jetson-up

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[BOOTSTRAP][ERROR] container is not running: ${CONTAINER}" >&2
  docker ps -a --format 'table {{.Names}}\t{{.Status}}'
  exit 20
fi

if [ "${FIELD_BUILD_IN_DOCKER}" = "auto" ] && ! docker exec "${CONTAINER}" bash -lc 'cd /ros2_ws/ros2_ws5 && test -f install_docker/setup.bash' >/dev/null 2>&1; then
  FIELD_BUILD_IN_DOCKER=true
fi

if [ "${FIELD_BUILD_IN_DOCKER}" = "true" ]; then
  docker exec "${CONTAINER}" bash -lc '
set -euo pipefail
cd /ros2_ws/ros2_ws5
source /opt/ros/humble/setup.bash
colcon --log-base log_docker build \
  --build-base build_docker \
  --install-base install_docker \
  --packages-select \
  ugv_description \
  ugv_tools \
  ugv_nav \
  waver_patrol \
  waver_seo_tracking \
  waver_experiment_logger
'
fi

docker exec "${CONTAINER}" bash -lc '
set -euo pipefail
cd /ros2_ws/ros2_ws5
source /opt/ros/humble/setup.bash
source install_docker/setup.bash
ros2 pkg prefix waver_patrol
ros2 pkg prefix ugv_tools
'
echo "WAVER_JETSON_BOOTSTRAP=PASS"
REMOTE
