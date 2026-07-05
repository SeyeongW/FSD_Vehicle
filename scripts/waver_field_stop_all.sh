#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"
REMOTE=false
while [ "$#" -gt 0 ]; do
  case "$1" in
    --remote) REMOTE=true; shift ;;
    --jetson-host) JETSON_HOST="$2"; shift 2 ;;
    --jetson-user) JETSON_USER="$2"; shift 2 ;;
    --docker-container|--container) CONTAINER="$2"; shift 2 ;;
    -h|--help) echo "Usage: bash scripts/waver_field_stop_all.sh [--remote] [--jetson-host HOST] [--jetson-user USER] [--container NAME]"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [ "${REMOTE}" = "true" ]; then
  # shellcheck source=scripts/waver_field_env_load.sh
  source "${ROOT}/scripts/waver_field_env_load.sh"
  waver_field_env_require
  waver_field_env_ensure_password
  waver_ssh_cmd
  "${WAVER_SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "bash -s" -- "${JETSON_WS}" "${CONTAINER}" <<'REMOTE_STOP'
set -euo pipefail
JETSON_WS="$1"
CONTAINER="$2"
DOCKER_SOURCE='if [ -f /opt/ros/humble/install/setup.bash ]; then source /opt/ros/humble/install/setup.bash; fi; source /opt/ros/humble/setup.bash; if [ -f install_docker/setup.bash ]; then source install_docker/setup.bash; elif [ -f install/setup.bash ]; then source install/setup.bash; fi'
for _ in $(seq 1 10); do
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true"
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 2 ros2 topic pub --once /waver/cmd_vel_safety geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true"
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 2 ros2 topic pub --once /waver/manual_cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true"
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 2 ros2 topic pub --once /waver/mission_command std_msgs/msg/String '{data: STOP}' >/dev/null 2>&1 || true"
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 2 ros2 topic pub --once /waver/sound_command std_msgs/msg/String '{data: CANCEL}' >/dev/null 2>&1 || true"
  sleep 0.05
done
for pat in nav2_container controller_server planner_server bt_navigator waver_base_driver_node serial_cmd_vel_bridge waver_cmd_vel_serial_bridge; do
  docker exec "${CONTAINER}" pkill -f "${pat}" 2>/dev/null || true
done
base_state="$(docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 3 ros2 topic echo --once --full-length /waver/base_driver_state 2>/dev/null || true")"
if grep -Eq 'left=0\.000|L.: 0\.0|x=0\.000' <<<"${base_state}" || [ -z "${base_state}" ]; then
  echo "STOP_CONFIRMED_REMOTE"
else
  echo "STOP_FAILED_REMOTE"
  echo "${base_state}"
  exit 1
fi
REMOTE_STOP
  exit 0
fi

source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f install/setup.bash ]; then source install/setup.bash; fi

ok=true
for _ in $(seq 1 8); do
  timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || ok=false
  timeout 2 ros2 topic pub --once /waver/cmd_vel_safety geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /waver/manual_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /waver/mission_command std_msgs/msg/String "{data: STOP}" >/dev/null 2>&1 || true
  timeout 2 ros2 topic pub --once /waver/sound_command std_msgs/msg/String "{data: CANCEL}" >/dev/null 2>&1 || true
  sleep 0.05
done

for pat in nav2_container controller_server planner_server bt_navigator waver_base_driver_node serial_cmd_vel_bridge waver_cmd_vel_serial_bridge; do
  pkill -f "${pat}" 2>/dev/null || true
done

if [ "${ok}" = "true" ]; then
  echo "STOP_CONFIRMED"
else
  echo "STOP_FAILED"
  exit 1
fi
