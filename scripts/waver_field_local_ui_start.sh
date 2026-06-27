#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=scripts/waver_field_env_load.sh
source scripts/waver_field_env_load.sh
waver_field_env_require
waver_field_env_ensure_password

source /opt/ros/humble/setup.bash
WAVER_LOCAL_UI_BUILD_IF_MISSING="${WAVER_LOCAL_UI_BUILD_IF_MISSING:-auto}"
if [ ! -f install/setup.bash ] && [ "${WAVER_LOCAL_UI_BUILD_IF_MISSING}" != "false" ]; then
  if ! command -v colcon >/dev/null 2>&1; then
    echo "[LOCAL][ERROR] install/setup.bash is missing and colcon is not installed." >&2
    echo "[LOCAL][ERROR] Run: bash scripts/waver_setup_local_pc.sh --install" >&2
    exit 2
  fi
  echo "[LOCAL] install/setup.bash missing; building local UI packages"
  colcon build --symlink-install --packages-up-to ugv_tools waver_patrol
fi
if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "[LOCAL][ERROR] install/setup.bash is missing. Build failed or was disabled." >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
WAVER_SKIP_JETSON_CHECK="${WAVER_SKIP_JETSON_CHECK:-false}"

select_jetson_host() {
  if [ "${WAVER_SKIP_JETSON_CHECK}" = "true" ]; then
    echo "[LOCAL][WARN] WAVER_SKIP_JETSON_CHECK=true; UI may open without a working Jetson bridge."
    return 0
  fi
  if [ "${JETSON_HOST_AUTO}" != "true" ]; then
    return 0
  fi
  local seen=" "
  local candidate
  for candidate in ${JETSON_HOST_CANDIDATES}; do
    [ -n "${candidate}" ] || continue
    case "${seen}" in
      *" ${candidate} "*) continue ;;
    esac
    seen="${seen}${candidate} "
    echo "[LOCAL] probing Jetson SSH ${JETSON_USER}@${candidate}"
    if ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 -p "${JETSON_PORT:-22}" \
      "${JETSON_USER}@${candidate}" "echo WAVER_JETSON_SSH_OK" >/tmp/waver_ui_jetson_probe.log 2>&1; then
      JETSON_HOST="${candidate}"
      printf '%s\n' "${JETSON_HOST}" > "${HOME}/.waver_jetson_host"
      echo "[LOCAL] selected Jetson host: ${JETSON_HOST}"
      return 0
    fi
    if [ -n "${JETSON_PASS:-}" ] && [ "${WAVER_ALLOW_PASSWORD_SSH:-}" = "1" ] && command -v sshpass >/dev/null 2>&1; then
      if sshpass -p "${JETSON_PASS}" ssh -o StrictHostKeyChecking=accept-new -o ConnectTimeout=5 -p "${JETSON_PORT:-22}" \
        "${JETSON_USER}@${candidate}" "echo WAVER_JETSON_SSH_OK" >/tmp/waver_ui_jetson_probe.log 2>&1; then
        JETSON_HOST="${candidate}"
        printf '%s\n' "${JETSON_HOST}" > "${HOME}/.waver_jetson_host"
        echo "[LOCAL] selected Jetson host by explicit password auth: ${JETSON_HOST}"
        return 0
      fi
    fi
    tail -3 /tmp/waver_ui_jetson_probe.log 2>/dev/null || true
  done
  echo "[LOCAL][ERROR] Jetson SSH bridge is not reachable; refusing to open a field UI that would only change local values." >&2
  echo "[LOCAL][ERROR] Candidates: ${JETSON_HOST_CANDIDATES}" >&2
  ip -4 addr show | sed 's/^/[LOCAL][NET] /' >&2 || true
  echo "[LOCAL][ERROR] Reconnect PC and Jetson to the same hotspot, then run:" >&2
  echo "[LOCAL][ERROR]   JETSON_HOST=<current_jetson_ip> bash scripts/waver_field_local_ui_start.sh" >&2
  echo "[LOCAL][ERROR] For Gazebo-only UI, set WAVER_SKIP_JETSON_CHECK=true." >&2
  exit 13
}

if [ -f "${HOME}/.waver_jetson_host" ] && [ "${JETSON_HOST_AUTO}" = "true" ]; then
  cached_host="$(head -n 1 "${HOME}/.waver_jetson_host" | tr -d '[:space:]')"
  if [ -n "${cached_host}" ]; then
    JETSON_HOST_CANDIDATES="${cached_host} ${JETSON_HOST_CANDIDATES}"
  fi
fi

select_jetson_host

echo "[LOCAL] starting remote panel bridge to ${JETSON_USER}@${JETSON_HOST} ws=${JETSON_WS}"

ROS_ARGS=(
  ros2 run ugv_tools waver_remote_panel --ros-args
  --params-file src/ugv_main/ugv_tools/config/waver_4wd_control.yaml \
  -p publish_direct_cmd_vel:=false \
  -p profile:=operator_bridge \
  -p remote_bridge_enabled:=true \
  -p remote_bridge_host:="${JETSON_HOST}" \
  -p remote_bridge_user:="${JETSON_USER}" \
  -p remote_bridge_port:="${JETSON_PORT:-22}" \
  -p remote_bridge_workspace:="${JETSON_WS}" \
  -p remote_bridge_use_docker:=true \
  -p remote_bridge_container:="${CONTAINER}" \
  -p remote_bridge_container_workspace:=/ros2_ws/ros2_ws5 \
  -p remote_bridge_setup_script:=install_docker/setup.bash \
  -p remote_bridge_ros_domain_id:="${ROS_DOMAIN_ID}" \
  -p remote_bridge_rmw:="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  -p remote_bridge_command_timeout_s:=0.12 \
  -p lidar_required:=false \
  -p enable_scan_assist:=false \
  -p allow_start_without_map:=true \
  -p allow_start_without_localization:=true \
  -p publish_mode_heartbeat:=false \
  -p manual_override_returns_to_auto:=false \
  -p command_rate_hz:=80.0 \
  -p key_release_debounce_ms:=100 \
  -p default_speed:=0.085 \
  -p default_angular:=0.16 \
  -p max_linear_speed:=0.12 \
  -p max_angular_speed:=0.16 \
  -p max_linear_accel:=2.00 \
  -p max_angular_accel:=2.50
)

if [ -n "${JETSON_PASS:-}" ]; then
  ROS_ARGS+=(-p remote_bridge_password:="${JETSON_PASS}")
fi
if [ -n "${JETSON_KEY_FILENAME:-}" ]; then
  ROS_ARGS+=(-p remote_bridge_key_filename:="${JETSON_KEY_FILENAME}")
fi

exec "${ROS_ARGS[@]}"
