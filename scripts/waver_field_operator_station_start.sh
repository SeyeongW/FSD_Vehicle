#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

DRY_RUN=false
START_RVIZ=true
START_UI=true
MAP_TOPIC="${WAVER_MAP_TOPIC:-/map}"
SCAN_TOPIC="${WAVER_SCAN_TOPIC:-/scan_safety}"
POINTCLOUD_TOPIC="${WAVER_POINTCLOUD_TOPIC:-/livox/lidar}"
FIXED_FRAME="${WAVER_FIXED_FRAME:-map}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --dry-run) DRY_RUN=true; shift ;;
    --rviz) START_RVIZ=true; shift ;;
    --ui) START_UI=true; shift ;;
    --rviz-only) START_RVIZ=true; START_UI=false; shift ;;
    --ui-only) START_RVIZ=false; START_UI=true; shift ;;
    --no-rviz) START_RVIZ=false; shift ;;
    --no-ui) START_UI=false; shift ;;
    --map-topic) MAP_TOPIC="$2"; shift 2 ;;
    --scan-topic) SCAN_TOPIC="$2"; shift 2 ;;
    --pointcloud-topic) POINTCLOUD_TOPIC="$2"; shift 2 ;;
    --fixed-frame) FIXED_FRAME="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
Usage: bash scripts/waver_field_operator_station_start.sh [options]

Starts the local operator station: RViz and/or Waver remote panel. It does not
start real robot backend nodes on the local PC. The backend must already be
running in Jetson Docker through waver_start_field_backend.sh or the approved
field backend script.

Options:
  --dry-run
  --rviz / --ui
  --rviz-only / --ui-only
  --no-rviz / --no-ui
  --map-topic TOPIC
  --scan-topic TOPIC
  --pointcloud-topic TOPIC
  --fixed-frame FRAME
EOF
      exit 0
      ;;
    *) echo "[OPERATOR_STATION][ERROR] unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [ "${START_RVIZ}" = "false" ] && [ "${START_UI}" = "false" ]; then
  echo "[OPERATOR_STATION][ERROR] both RViz and UI are disabled." >&2
  exit 2
fi

if [ "${DRY_RUN}" = "true" ]; then
  python3 scripts/waver_check_local_operator_deps.py --check || true
else
  python3 scripts/waver_check_local_operator_deps.py --mode rviz --require-ros
fi

# shellcheck source=scripts/waver_field_env_load.sh
source "${ROOT}/scripts/waver_field_env_load.sh"
if [ "${DRY_RUN}" = "false" ]; then
  waver_field_env_require
  waver_field_env_ensure_password
else
  missing_env=()
  for name in JETSON_HOST JETSON_USER JETSON_WS CONTAINER ROS_DOMAIN_ID SERIAL_PORT; do
    value="${!name:-}"
    if _waver_is_placeholder "${value}"; then
      missing_env+=("${name}")
    fi
  done
fi

if [ "${DRY_RUN}" = "false" ] && [ "${WAVER_SKIP_JETSON_CHECK:-false}" != "true" ]; then
  waver_ssh_cmd
  echo "[OPERATOR_STATION] checking Jetson Docker backend ${JETSON_USER}@${JETSON_HOST}:${JETSON_PORT:-22} container=${CONTAINER}"
  "${WAVER_SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" \
    "docker ps --format '{{.Names}}' | grep -qx '${CONTAINER}' && docker exec '${CONTAINER}' bash -lc 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}; export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}; source /opt/ros/humble/setup.bash >/dev/null 2>&1 || source /opt/ros/humble/install/setup.bash >/dev/null 2>&1 || true; ros2 node list >/tmp/waver_operator_station_nodes.txt 2>&1; grep -Eq \"(safety_cmd_mux_node|waver_base_driver_node|mission_patrol_manager_node)\" /tmp/waver_operator_station_nodes.txt'" \
    >/tmp/waver_operator_station_jetson_check.log 2>&1 || {
      echo "[OPERATOR_STATION][ERROR] Jetson Docker backend is not ready." >&2
      tail -40 /tmp/waver_operator_station_jetson_check.log >&2 || true
      echo "[OPERATOR_STATION][ERROR] Start backend first, then rerun this script." >&2
      exit 13
    }
fi

RVIZ_CMD=(bash scripts/waver_field_rviz_start.sh
  --map-topic "${MAP_TOPIC}"
  --scan-topic "${SCAN_TOPIC}"
  --pointcloud-topic "${POINTCLOUD_TOPIC}"
  --fixed-frame "${FIXED_FRAME}"
)
UI_CMD=(bash scripts/waver_field_local_ui_start.sh)

if [ "${DRY_RUN}" = "true" ]; then
  echo "OPERATOR_STATION_DRY_RUN=1"
  if [ "${#missing_env[@]}" -gt 0 ]; then
    echo "DRY_RUN_MISSING_ENV=${missing_env[*]}"
  fi
  if [ "${START_RVIZ}" = "true" ]; then
    printf 'RVIZ_CMD='
    printf '%q ' "${RVIZ_CMD[@]}"
    printf '\n'
  fi
  if [ "${START_UI}" = "true" ]; then
    printf 'UI_CMD='
    printf '%q ' "${UI_CMD[@]}"
    printf '\n'
  fi
  exit 0
fi

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill "${pid}" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT INT TERM

if [ "${START_RVIZ}" = "true" ]; then
  if [ "${START_UI}" = "true" ]; then
    "${RVIZ_CMD[@]}" &
    pids+=("$!")
    sleep 1
  else
    exec "${RVIZ_CMD[@]}"
  fi
fi
if [ "${START_UI}" = "true" ]; then
  exec "${UI_CMD[@]}"
else
  exec bash scripts/waver_field_rviz_start.sh
fi
