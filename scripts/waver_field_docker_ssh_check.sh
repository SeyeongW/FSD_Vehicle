#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"

CYCLES=1
LOCAL_COMPOSE=false
JETSON_COMPOSE=false
NO_MOTION=true
REPORT_ROOT="${WAVER_FIELD_CHECK_REPORT_ROOT:-${ROOT}/reports/field_docker_ssh_check/$(date +%Y%m%d_%H%M%S)}"

usage() {
  cat <<'EOF'
Usage: bash scripts/waver_field_docker_ssh_check.sh [--local-compose] [--jetson-compose] [--no-motion] [--cycles N]

Default behavior is no-motion status validation. Jetson SSH checks are skipped
unless WAVER_ENABLE_SSH_VALIDATION=1 is set. This script does not start backend
nodes and does not publish motion commands.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --local-compose) LOCAL_COMPOSE=true; shift ;;
    --jetson-compose) JETSON_COMPOSE=true; shift ;;
    --no-motion) NO_MOTION=true; shift ;;
    --cycles) CYCLES="${2:?missing cycles}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

mkdir -p "${REPORT_ROOT}"
summary="${REPORT_ROOT}/summary.csv"
echo "cycle,status,key_output,evidence" > "${summary}"

run_cycle() {
  local cycle="$1"
  local dir="${REPORT_ROOT}/cycle_${cycle}"
  mkdir -p "${dir}"
  local log="${dir}/field_docker_ssh_check.log"
  local status="PASS"
  local key="no_motion_status_ok"

  {
    echo "FIELD_DOCKER_SSH_CHECK cycle=${cycle}"
    echo "NO_MOTION=${NO_MOTION}"
    waver_field_env_require
    waver_field_env_masked_summary

    if [ "${LOCAL_COMPOSE}" = "true" ]; then
      if command -v docker >/dev/null 2>&1; then
        docker compose -f "${ROOT}/docker-compose.jetson.yml" config >/tmp/waver_field_check_compose_${cycle}.yaml
        echo "LOCAL_COMPOSE_CONFIG=PASS"
      else
        echo "LOCAL_COMPOSE_CONFIG=SKIP_WITH_REASON docker command not found"
      fi
    fi

    if [ "${JETSON_COMPOSE}" = "true" ]; then
      if [ "${WAVER_ENABLE_SSH_VALIDATION:-0}" != "1" ]; then
        echo "JETSON_SSH=SKIP_WITH_REASON set WAVER_ENABLE_SSH_VALIDATION=1 for live Jetson status check"
      else
        waver_field_env_ensure_password
        waver_ssh_cmd
        "${WAVER_SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "
          set -e
          echo JETSON_HOSTNAME=\$(hostname)
          test -d '${JETSON_WS}'
          cd '${JETSON_WS}'
          test -f docker-compose.jetson.yml
          if command -v docker >/dev/null 2>&1; then
            docker ps --format '{{.Names}} {{.Status}}' | sed 's/^/DOCKER_PS /'
            docker compose -f docker-compose.jetson.yml config >/tmp/waver_field_check_compose.yaml
            echo JETSON_COMPOSE_CONFIG=PASS
          else
            echo JETSON_DOCKER=SKIP_WITH_REASON docker command not found
          fi
          echo NO_MOTION_CHECK=PASS
        "
      fi
    fi

    echo "FIELD_DOCKER_SSH_CHECK=PASS"
  } >"${log}" 2>&1 || {
    status="FAIL"
    key="$(tail -1 "${log}" | tr ',' ';')"
  }
  if grep -q "SKIP_WITH_REASON" "${log}" && [ "${status}" = "PASS" ]; then
    key="pass_with_skips"
  fi
  echo "${cycle},${status},${key},${log}" >> "${summary}"
  echo "FIELD_DOCKER_SSH_CHECK_${cycle}=${status} evidence=${log}"
  [ "${status}" = "PASS" ]
}

overall=0
for cycle in $(seq 1 "${CYCLES}"); do
  run_cycle "${cycle}" || overall=1
done

echo "FIELD_DOCKER_SSH_REPORT=${REPORT_ROOT}"
if [ "${overall}" -eq 0 ]; then
  echo "FIELD_DOCKER_SSH_CHECK=PASS"
else
  echo "FIELD_DOCKER_SSH_CHECK=FAIL"
fi
exit "${overall}"
