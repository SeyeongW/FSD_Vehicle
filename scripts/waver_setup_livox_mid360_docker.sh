#!/usr/bin/env bash
set -euo pipefail

# Prepare the official Livox-SDK2 + livox_ros_driver2 stack in the Jetson
# Docker container used by the Waver field backend.  This script deliberately
# does not change the validated local-PC -> SSH -> Jetson -> Docker -> Waver
# USB serial control path; it only makes the Mid-360 ROS driver available.

LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${LOCAL_ROOT}/scripts/waver_field_env_load.sh"
waver_field_env_require
waver_ssh_cmd
waver_scp_cmd
SSH_CMD=("${WAVER_SSH_CMD[@]}")
SCP_CMD=("${WAVER_SCP_CMD[@]}")
JETSON_HOST_CANDIDATES="${JETSON_HOST_CANDIDATES:-${JETSON_HOST}}"
LIVOX_SDK2_URL="${LIVOX_SDK2_URL:-https://github.com/Livox-SDK/Livox-SDK2.git}"
LIVOX_DRIVER2_URL="${LIVOX_DRIVER2_URL:-https://github.com/Livox-SDK/livox_ros_driver2.git}"
BUILD_LIVOX_SDK2="${BUILD_LIVOX_SDK2:-true}"

select_jetson_host() {
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
    if "${SSH_CMD[@]}" "${JETSON_USER}@${candidate}" "echo WAVER_JETSON_SSH_OK" >/tmp/waver_livox_jetson_probe.log 2>&1; then
      JETSON_HOST="${candidate}"
      printf '%s\n' "${JETSON_HOST}" > "${HOME}/.waver_jetson_host"
      echo "[LOCAL] selected Jetson host: ${JETSON_HOST}"
      return 0
    fi
    tail -3 /tmp/waver_livox_jetson_probe.log 2>/dev/null || true
  done
  echo "[LOCAL][ERROR] Could not reach Jetson SSH on candidates: ${JETSON_HOST_CANDIDATES}" >&2
  exit 12
}

if [ -f "${HOME}/.waver_jetson_host" ] && [ "${JETSON_HOST_AUTO}" = "true" ]; then
  cached_host="$(head -n 1 "${HOME}/.waver_jetson_host" | tr -d '[:space:]')"
  if [ -n "${cached_host}" ]; then
    JETSON_HOST_CANDIDATES="${cached_host} ${JETSON_HOST_CANDIDATES}"
  fi
fi

mkdir -p "${LOCAL_ROOT}/src"
if ! git -C "${LOCAL_ROOT}/src/Livox-SDK2" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if [ -e "${LOCAL_ROOT}/src/Livox-SDK2" ]; then
    echo "[LOCAL][ERROR] ${LOCAL_ROOT}/src/Livox-SDK2 exists but is not a git checkout." >&2
    echo "[LOCAL][ERROR] Move it aside or replace it with ${LIVOX_SDK2_URL}." >&2
    exit 13
  fi
  echo "[LOCAL] cloning official Livox-SDK2"
  git clone "${LIVOX_SDK2_URL}" "${LOCAL_ROOT}/src/Livox-SDK2"
fi
if ! git -C "${LOCAL_ROOT}/src/livox_ros_driver2" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  if [ -e "${LOCAL_ROOT}/src/livox_ros_driver2" ]; then
    echo "[LOCAL][ERROR] ${LOCAL_ROOT}/src/livox_ros_driver2 exists but is not a git checkout." >&2
    echo "[LOCAL][ERROR] Move it aside or replace it with ${LIVOX_DRIVER2_URL}." >&2
    exit 14
  fi
  echo "[LOCAL] cloning official livox_ros_driver2"
  git clone "${LIVOX_DRIVER2_URL}" "${LOCAL_ROOT}/src/livox_ros_driver2"
fi

select_jetson_host

echo "[LOCAL] syncing Livox sources to ${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src"
"${SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "mkdir -p '${JETSON_WS}/src'"
for rel in src/Livox-SDK2 src/livox_ros_driver2; do
  "${SCP_CMD[@]}" -r "${LOCAL_ROOT}/${rel}" "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/src/" >/dev/null
done

"${SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "bash -s" -- \
  "${JETSON_WS}" "${CONTAINER}" "${BUILD_LIVOX_SDK2}" <<'REMOTE'
set -euo pipefail

JETSON_WS="$1"
CONTAINER="$2"
BUILD_LIVOX_SDK2="$3"

cd "${JETSON_WS}"
echo "[JETSON] repo=$(pwd)"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON] starting Docker container via docker/run.sh jetson-up"
  bash docker/run.sh jetson-up >/tmp/waver_livox_docker_run.log 2>&1 || {
    echo "[JETSON][ERROR] docker/run.sh jetson-up failed. See /tmp/waver_livox_docker_run.log" >&2
    tail -80 /tmp/waver_livox_docker_run.log || true
    exit 20
  }
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON][ERROR] Docker container ${CONTAINER} is not running" >&2
  docker ps -a --format 'table {{.Names}}\t{{.Status}}'
  exit 21
fi

echo "[JETSON] copying official Livox sources into Docker workspace"
docker exec "${CONTAINER}" mkdir -p /ros2_ws/ros2_ws5/src
docker exec "${CONTAINER}" rm -rf /ros2_ws/ros2_ws5/src/Livox-SDK2 /ros2_ws/ros2_ws5/src/livox_ros_driver2
docker cp "${JETSON_WS}/src/Livox-SDK2" "${CONTAINER}:/ros2_ws/ros2_ws5/src/" >/dev/null
docker cp "${JETSON_WS}/src/livox_ros_driver2" "${CONTAINER}:/ros2_ws/ros2_ws5/src/" >/dev/null

echo "[JETSON] normalizing livox_ros_driver2 ROS 2 package files"
docker exec "${CONTAINER}" bash -lc '
set -euo pipefail
cd /ros2_ws/ros2_ws5/src/livox_ros_driver2
if [ -f package_ROS2.xml ]; then
  cp package_ROS2.xml package.xml
fi
if [ -d launch_ROS2 ]; then
  rm -rf launch
  cp -r launch_ROS2 launch
fi
python3 - <<'"'"'PY'"'"'
from pathlib import Path

p = Path("CMakeLists.txt")
text = p.read_text()

if "Default it from ROS_DISTRO" not in text:
    text = text.replace(
        "  project(livox_ros_driver2)\n\n  # Default to C99",
        "  project(livox_ros_driver2)\n\n"
        "  # ROS 2 Humble users normally run plain `colcon build` without passing the\n"
        "  # vendor build flag `-DHUMBLE_ROS=humble`. Default it from ROS_DISTRO so the\n"
        "  # Humble rosidl branch is used in workspace builds.\n"
        "  if(NOT DEFINED HUMBLE_ROS AND \"$ENV{ROS_DISTRO}\" STREQUAL \"humble\")\n"
        "    set(HUMBLE_ROS \"humble\")\n"
        "  endif()\n\n"
        "  # Default to C99",
    )

if "set(LIVOX_INTERFACE_TARGET \"${cpp_typesupport_target}\")" not in text:
    text = text.replace(
        "    rosidl_get_typesupport_target(cpp_typesupport_target\n"
        "    ${LIVOX_INTERFACES} \"rosidl_typesupport_cpp\")\n"
        "    target_link_libraries(${PROJECT_NAME} \"${cpp_typesupport_target}\")\n"
        "  else()",
        "    rosidl_get_typesupport_target(cpp_typesupport_target\n"
        "    ${LIVOX_INTERFACES} \"rosidl_typesupport_cpp\")\n"
        "    target_link_libraries(${PROJECT_NAME} \"${cpp_typesupport_target}\")\n"
        "    set(LIVOX_INTERFACE_TARGET \"${cpp_typesupport_target}\")\n"
        "    set(LIVOX_INTERFACES_INCLUDE_DIRECTORIES \"\")\n"
        "  else()",
    )

p.write_text(text)
PY
'

echo "[JETSON] building Livox driver in a clean Humble environment"
BUILD_SELECT="livox_ros_driver2"
if [ "${BUILD_LIVOX_SDK2}" = "true" ]; then
  BUILD_SELECT="livox_sdk2 livox_ros_driver2"
fi

docker exec "${CONTAINER}" bash -lc "
env -i HOME=/root USER=root PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LD_LIBRARY_PATH=/usr/local/lib bash -lc '
set -eo pipefail
cd /ros2_ws/ros2_ws5
source /opt/ros/humble/setup.bash
colcon --log-base log_docker build \
  --build-base build_docker \
  --install-base install_docker \
  --packages-select ${BUILD_SELECT} \
  --event-handlers console_direct+
'
"

echo "[JETSON] verifying Livox ROS package and message interface"
docker exec "${CONTAINER}" bash -lc '
set -eo pipefail
cd /ros2_ws/ros2_ws5
if [ -f /opt/ros/humble/install/setup.bash ]; then
  source /opt/ros/humble/install/setup.bash
fi
source /opt/ros/humble/setup.bash
source install_docker/setup.bash
ros2 pkg prefix livox_ros_driver2
ros2 interface show livox_ros_driver2/msg/CustomMsg >/tmp/waver_livox_custom_msg.txt
ros2 launch livox_ros_driver2 msg_MID360_launch.py --show-args >/tmp/waver_livox_mid360_launch_args.txt
cat /tmp/waver_livox_mid360_launch_args.txt
'

echo "[JETSON] current Jetson IPv4 addresses:"
ip -4 addr show | sed "s/^/[JETSON][NET] /" || true
echo "[JETSON] Livox Mid-360 default config expects Jetson host IP 192.168.1.5 and sensor IP 192.168.1.12."
echo "[JETSON] LIVOX_MID360_DOCKER_READY=YES"
REMOTE

echo "[LOCAL] LIVOX_MID360_DOCKER_READY=YES"
echo "[LOCAL] Real LiDAR/Nav2 backend command:"
echo "  cd ~/ros2_ws5/FSD_Vehicle"
echo "  START_LIVOX_DRIVER=true bash scripts/waver_field_lidar_nav_backend_start.sh"
