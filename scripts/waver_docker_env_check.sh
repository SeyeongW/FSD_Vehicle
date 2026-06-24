#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"

if ! command -v docker >/dev/null 2>&1; then
  echo "WAVER_DOCKER_ENV_CHECK=SKIP docker command not found"
  exit 0
fi

cd "${ROOT}"
docker compose -f docker-compose.jetson.yml config >/tmp/waver_docker_compose_jetson_config.yaml
echo "COMPOSE_CONFIG_OK=docker-compose.jetson.yml"

if docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "CONTAINER_RUNNING=${CONTAINER}"
  docker exec "${CONTAINER}" bash -lc 'test -f /opt/ros/humble/setup.bash && echo ROS_HUMBLE_OK'
else
  echo "CONTAINER_RUNNING=NO (${CONTAINER})"
fi

echo "WAVER_DOCKER_ENV_CHECK=PASS"
