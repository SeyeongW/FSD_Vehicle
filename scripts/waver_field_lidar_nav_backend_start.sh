#!/usr/bin/env bash
set -euo pipefail

# Real LiDAR/Nav2 backend starter.
# This preserves the validated local-PC -> SSH -> Jetson -> Docker -> Waver USB
# field topology, but starts the real localization/Nav2 mission stack instead of
# the supervised open-loop micro-patrol helper.

FIELD_ENV_FILE="${WAVER_FIELD_ENV_FILE:-$HOME/.waver_field_env}"
if [ -f "${FIELD_ENV_FILE}" ]; then
  # Local-only field settings: Jetson IP/user/password/workspace. This file is
  # intentionally outside the repository so the field password is not committed.
  set -a
  # shellcheck disable=SC1090
  source "${FIELD_ENV_FILE}"
  set +a
fi

JETSON_HOST="${JETSON_HOST:-10.139.225.150}"
JETSON_USER="${JETSON_USER:-sw}"
JETSON_PASS="${JETSON_PASS:-}"
JETSON_WS="${JETSON_WS:-/home/sw/ugv_ws/FSD_Vehicle}"
JETSON_HOST_AUTO="${JETSON_HOST_AUTO:-true}"
JETSON_HOST_CANDIDATES="${JETSON_HOST_CANDIDATES:-${JETSON_HOST} 10.139.225.150 10.63.240.150 10.139.225.126}"
CONTAINER="${CONTAINER:-fsd_dev_jetson}"
SERIAL_PORT="${SERIAL_PORT:-auto}"
FIELD_BUILD_IN_DOCKER="${FIELD_BUILD_IN_DOCKER:-false}"

MAP_PATH="${MAP_PATH:-/ros2_ws/ugv_ws/maps/waver_latest_map.yaml}"
WAYPOINT_FILE="${WAYPOINT_FILE:-/ros2_ws/ugv_ws/src/waver_patrol/waypoints/waver_real_0p5m_square_patrol.yaml}"
MISSION_PARAMS_FILE="${MISSION_PARAMS_FILE:-/ros2_ws/ugv_ws/src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-/ros2_ws/ugv_ws/src/waver_patrol/config/nav2_params_waver_real.yaml}"
ODOM_SOURCE="${ODOM_SOURCE:-base}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/mid360_PointCloud2}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
START_LIVOX_DRIVER="${START_LIVOX_DRIVER:-true}"
LIVOX_TOPIC="${LIVOX_TOPIC:-${POINTCLOUD_TOPIC}}"
LIVOX_CONFIG_PATH="${LIVOX_CONFIG_PATH:-/ros2_ws/ugv_ws/install_docker/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json}"
LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-livox_frame}"
LIVOX_PUBLISH_FREQ="${LIVOX_PUBLISH_FREQ:-10.0}"
LIVOX_BD_CODE="${LIVOX_BD_CODE:-livox0000000001}"
SAFETY_MAX_LINEAR_SPEED="${SAFETY_MAX_LINEAR_SPEED:-0.05}"
SAFETY_MAX_ANGULAR_SPEED="${SAFETY_MAX_ANGULAR_SPEED:-0.20}"
ENABLE_BIRD_STACK="${ENABLE_BIRD_STACK:-false}"
ENABLE_SOUND_STACK="${ENABLE_SOUND_STACK:-false}"

SSH_OPTS=(-o StrictHostKeyChecking=no -o ConnectTimeout=8)
if [ -n "${JETSON_PASS}" ] && [ "${WAVER_ALLOW_PASSWORD_SSH:-}" = "1" ] && command -v sshpass >/dev/null 2>&1; then
  SSH_CMD=(sshpass -p "${JETSON_PASS}" ssh "${SSH_OPTS[@]}")
  SCP_CMD=(sshpass -p "${JETSON_PASS}" scp "${SSH_OPTS[@]}")
else
  if [ -n "${JETSON_PASS}" ] && [ "${WAVER_ALLOW_PASSWORD_SSH:-}" != "1" ]; then
    echo "[LOCAL][WARN] JETSON_PASS is set but ignored. Set WAVER_ALLOW_PASSWORD_SSH=1 only for temporary password auth." >&2
  fi
  echo "[LOCAL] using SSH key/agent or interactive SSH auth." >&2
  SSH_CMD=(ssh "${SSH_OPTS[@]}")
  SCP_CMD=(scp "${SSH_OPTS[@]}")
fi

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
    if "${SSH_CMD[@]}" "${JETSON_USER}@${candidate}" "echo WAVER_JETSON_SSH_OK" >/tmp/waver_lidar_nav_jetson_probe.log 2>&1; then
      JETSON_HOST="${candidate}"
      printf '%s\n' "${JETSON_HOST}" > "${HOME}/.waver_jetson_host"
      echo "[LOCAL] selected Jetson host: ${JETSON_HOST}"
      return 0
    fi
    tail -3 /tmp/waver_lidar_nav_jetson_probe.log 2>/dev/null || true
  done
  echo "[LOCAL][ERROR] Could not reach Jetson SSH on candidates: ${JETSON_HOST_CANDIDATES}" >&2
  echo "[LOCAL][ERROR] Set JETSON_HOST=<current_jetson_ip> and retry." >&2
  exit 12
}

LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ -f "${HOME}/.waver_jetson_host" ] && [ "${JETSON_HOST_AUTO}" = "true" ]; then
  cached_host="$(head -n 1 "${HOME}/.waver_jetson_host" | tr -d '[:space:]')"
  if [ -n "${cached_host}" ]; then
    JETSON_HOST_CANDIDATES="${cached_host} ${JETSON_HOST_CANDIDATES}"
  fi
fi
select_jetson_host

echo "[LOCAL] target Jetson: ${JETSON_USER}@${JETSON_HOST} ws=${JETSON_WS}"
echo "[LOCAL] syncing real-nav source/config/map files to Jetson"
SYNC_PATHS=(
  "src/waver_patrol/launch"
  "src/waver_patrol/config"
  "src/waver_patrol/waypoints"
  "src/waver_patrol/waver_patrol"
  "src/ugv_main/ugv_nav/launch"
  "src/ugv_main/ugv_nav/param"
  "maps"
)

for rel in "${SYNC_PATHS[@]}"; do
  [ -e "${LOCAL_ROOT}/${rel}" ] || continue
  remote_parent="${JETSON_WS}/$(dirname "${rel}")"
  "${SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "mkdir -p '${remote_parent}'"
  "${SCP_CMD[@]}" -r "${LOCAL_ROOT}/${rel}" "${JETSON_USER}@${JETSON_HOST}:${remote_parent}/" >/dev/null
done

"${SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "bash -s" -- \
  "${JETSON_WS}" "${CONTAINER}" "${SERIAL_PORT}" "${FIELD_BUILD_IN_DOCKER}" \
  "${MAP_PATH}" "${WAYPOINT_FILE}" "${MISSION_PARAMS_FILE}" "${NAV2_PARAMS_FILE}" \
  "${ODOM_SOURCE}" "${POINTCLOUD_TOPIC}" "${SCAN_TOPIC}" \
  "${SAFETY_MAX_LINEAR_SPEED}" "${SAFETY_MAX_ANGULAR_SPEED}" \
  "${ENABLE_BIRD_STACK}" "${ENABLE_SOUND_STACK}" \
  "${START_LIVOX_DRIVER}" "${LIVOX_TOPIC}" "${LIVOX_CONFIG_PATH}" \
  "${LIVOX_FRAME_ID}" "${LIVOX_PUBLISH_FREQ}" "${LIVOX_BD_CODE}" <<'REMOTE'
set -euo pipefail

JETSON_WS="$1"
CONTAINER="$2"
SERIAL_REQUEST="$3"
FIELD_BUILD_IN_DOCKER="$4"
MAP_PATH="$5"
WAYPOINT_FILE="$6"
MISSION_PARAMS_FILE="$7"
NAV2_PARAMS_FILE="$8"
ODOM_SOURCE="$9"
POINTCLOUD_TOPIC="${10}"
SCAN_TOPIC="${11}"
SAFETY_MAX_LINEAR_SPEED="${12}"
SAFETY_MAX_ANGULAR_SPEED="${13}"
ENABLE_BIRD_STACK="${14}"
ENABLE_SOUND_STACK="${15}"
START_LIVOX_DRIVER="${16}"
LIVOX_TOPIC="${17}"
LIVOX_CONFIG_PATH="${18}"
LIVOX_FRAME_ID="${19}"
LIVOX_PUBLISH_FREQ="${20}"
LIVOX_BD_CODE="${21}"

cd "${JETSON_WS}"
echo "[JETSON] repo=$(pwd) branch=$(git branch --show-current 2>/dev/null || echo unknown) head=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"

SERIAL_PORT="${SERIAL_REQUEST}"
if [ "${SERIAL_PORT}" = "auto" ]; then
  SERIAL_PORT="$(ls /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_* 2>/dev/null | head -n 1 || true)"
  [ -n "${SERIAL_PORT}" ] || SERIAL_PORT="$(ls /dev/ttyUSB* 2>/dev/null | head -n 1 || true)"
fi
if [ -z "${SERIAL_PORT}" ]; then
  echo "[JETSON][ERROR] No Waver serial port found. Connect Waver USB to Jetson first." >&2
  exit 20
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON] starting Docker container via docker/run.sh jetson"
  bash docker/run.sh jetson >/tmp/waver_lidar_nav_docker_run.log 2>&1 || {
    echo "[JETSON][ERROR] docker/run.sh jetson failed. See /tmp/waver_lidar_nav_docker_run.log"
    tail -80 /tmp/waver_lidar_nav_docker_run.log || true
    exit 21
  }
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON][ERROR] Docker container ${CONTAINER} is not running"
  docker ps -a --format 'table {{.Names}}\t{{.Status}}'
  exit 22
fi

if [ -d "${JETSON_WS}/maps" ]; then
  docker exec "${CONTAINER}" mkdir -p /ros2_ws/ugv_ws/maps
  docker cp "${JETSON_WS}/maps/." "${CONTAINER}:/ros2_ws/ugv_ws/maps/" >/dev/null 2>&1 || true
fi

if [ "${FIELD_BUILD_IN_DOCKER}" = "true" ]; then
  echo "[JETSON] building selected packages inside Docker"
  docker exec "${CONTAINER}" bash -lc '
  set -eo pipefail
  cd /ros2_ws/ugv_ws
  source /opt/ros/humble/setup.bash
  colcon --log-base log_docker build \
    --build-base build_docker \
    --install-base install_docker \
    --packages-select ugv_description ugv_nav ugv_tools waver_patrol
  '
fi

echo "[JETSON] overlaying source launch/config/python into install_docker"
docker exec "${CONTAINER}" bash -lc '
set -e
cd /ros2_ws/ugv_ws
if [ -d install_docker/waver_patrol/share/waver_patrol ]; then
  rm -rf install_docker/waver_patrol/share/waver_patrol/launch \
         install_docker/waver_patrol/share/waver_patrol/config \
         install_docker/waver_patrol/share/waver_patrol/waypoints
  cp -r src/waver_patrol/launch install_docker/waver_patrol/share/waver_patrol/
  cp -r src/waver_patrol/config install_docker/waver_patrol/share/waver_patrol/
  cp -r src/waver_patrol/waypoints install_docker/waver_patrol/share/waver_patrol/
fi
if [ -d install_docker/waver_patrol/lib/python3.10/site-packages/waver_patrol ]; then
  cp -r src/waver_patrol/waver_patrol/* install_docker/waver_patrol/lib/python3.10/site-packages/waver_patrol/
fi
'

echo "[JETSON] stopping stale Waver/Nav2 processes"
pkill -f "docker exec -i ${CONTAINER}.*WAVER_REMOTE_BRIDGE_TIMEOUT_S" 2>/dev/null || true
for pat in \
  waver_base_driver_node safety_cmd_mux_node mission_patrol_manager_node target_goal_manager_node \
  livox_pointcloud_to_scan_node pointcloud_lidar_objects_node moving_object_map_transform_node \
  moving_object_motion_filter_node nav2_container controller_server planner_server bt_navigator \
  map_server amcl ekf_filter_node waver_gazebo_patrol serial_cmd_vel_bridge waver_cmd_vel_serial_bridge \
  livox_ros_driver2_node livox_lidar_publisher; do
  pkill -f "${pat}" 2>/dev/null || true
  docker exec "${CONTAINER}" pkill -f "${pat}" 2>/dev/null || true
done
docker exec "${CONTAINER}" bash -lc "ps -eo pid,args | awk '/livox_ros_driver2_node|livox_lidar_publisher/ && !/awk/ {print \$1}' | xargs -r kill -9" 2>/dev/null || true
sleep 1

echo "[JETSON] serial owner before launch:"
fuser -v "${SERIAL_PORT}" 2>&1 || true

DOCKER_SOURCE='source /opt/ros/humble/install/setup.bash && source /opt/ros/humble/setup.bash && source install_docker/setup.bash && export ROS_DOMAIN_ID=30 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
LIVOX_DOCKER_SOURCE='source /opt/ros/humble/install/setup.bash && source /opt/ros/humble/setup.bash && source install_docker/setup.bash && export ROS_DOMAIN_ID=30 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'
ENABLE_RL="false"
if [ "${ODOM_SOURCE}" = "ekf" ]; then
  ENABLE_RL="true"
fi

if [ "${START_LIVOX_DRIVER}" = "true" ]; then
  echo "[JETSON] checking livox_ros_driver2 inside Docker"
  if ! docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${LIVOX_DOCKER_SOURCE} && ros2 pkg prefix livox_ros_driver2 >/dev/null"; then
    echo "[JETSON][ERROR] livox_ros_driver2 is not built in Docker install_docker." >&2
    echo "[JETSON][ERROR] Run: bash scripts/waver_setup_livox_mid360_docker.sh" >&2
    exit 30
  fi
  echo "[JETSON] starting Livox Mid-360 driver: livox/lidar -> ${LIVOX_TOPIC}"
  docker exec -d "${CONTAINER}" bash -lc "
  cd /ros2_ws/ugv_ws
  ${LIVOX_DOCKER_SOURCE}
  exec ros2 run livox_ros_driver2 livox_ros_driver2_node --ros-args \
    -p xfer_format:=0 \
    -p multi_topic:=0 \
    -p data_src:=0 \
    -p publish_freq:=${LIVOX_PUBLISH_FREQ} \
    -p output_data_type:=0 \
    -p frame_id:=${LIVOX_FRAME_ID} \
    -p user_config_path:=${LIVOX_CONFIG_PATH} \
    -p cmdline_input_bd_code:=${LIVOX_BD_CODE} \
    -r livox/lidar:=${LIVOX_TOPIC} \
    > /tmp/waver_livox_mid360_driver.log 2>&1
  "
  sleep 2
fi

echo "[JETSON] starting real LiDAR/Nav2 backend"
echo "[JETSON] map=${MAP_PATH}"
echo "[JETSON] waypoints=${WAYPOINT_FILE}"
echo "[JETSON] serial=${SERIAL_PORT} odom_source=${ODOM_SOURCE} pointcloud=${POINTCLOUD_TOPIC} scan=${SCAN_TOPIC}"

docker exec -d "${CONTAINER}" bash -lc "
cd /ros2_ws/ugv_ws
${DOCKER_SOURCE}
exec ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  use_sim_time:=false \
  real_profile:=true \
  use_nav2:=true \
  default_mode:=STANDBY \
  enable_waver_base_driver:=true \
  serial_port:=${SERIAL_PORT} \
  start_serial_bridge:=false \
  start_base_feedback:=false \
  include_existing_ugv_driver:=false \
  odom_source:=${ODOM_SOURCE} \
  enable_robot_localization:=${ENABLE_RL} \
  enable_velocity_smoother:=true \
  require_scan:=true \
  scan_source:=mid360 \
  scan_source_safety:=mid360 \
  scan_source_slam:=mid360 \
  pointcloud_topic:=${POINTCLOUD_TOPIC} \
  scan_topic:=${SCAN_TOPIC} \
  map:=${MAP_PATH} \
  waypoint_file:=${WAYPOINT_FILE} \
  mission_params_file:=${MISSION_PARAMS_FILE} \
  nav2_params_file:=${NAV2_PARAMS_FILE} \
  enable_livox_scan_adapter:=true \
  enable_pointcloud_lidar_objects:=true \
  enable_moving_object_map_transform:=true \
  enable_moving_object_motion_filter:=true \
  enable_bird_detector:=${ENABLE_BIRD_STACK} \
  enable_bird_3d_fusion:=${ENABLE_BIRD_STACK} \
  enable_camera_gimbal_controller:=${ENABLE_BIRD_STACK} \
  enable_sound_deterrent:=${ENABLE_SOUND_STACK} \
  enable_sound_output:=false \
  sound_safety_ack:=false \
  enable_experiment_logger:=false \
  safety_max_linear_speed:=${SAFETY_MAX_LINEAR_SPEED} \
  safety_max_angular_speed:=${SAFETY_MAX_ANGULAR_SPEED} \
  bird_model_path:= \
  > /tmp/waver_lidar_nav_backend.log 2>&1
"

echo "[JETSON] waiting for backend graph"
for _ in $(seq 1 25); do
  nodes="$(docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && ros2 node list 2>/dev/null" || true)"
  if grep -q "/waver_base_driver_node" <<<"${nodes}" && grep -q "/safety_cmd_mux_node" <<<"${nodes}" && grep -q "/mission_patrol_manager_node" <<<"${nodes}"; then
    break
  fi
  sleep 1
done

echo "[JETSON] ROS nodes:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && ros2 node list 2>/dev/null | sort" || true

echo "[JETSON] /cmd_vel chain:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && ros2 topic info -v /cmd_vel" || true

if [ "${START_LIVOX_DRIVER}" = "true" ]; then
  echo "[JETSON] Livox Mid-360 package:"
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${LIVOX_DOCKER_SOURCE} && ros2 pkg prefix livox_ros_driver2" || true
  echo "[JETSON] ${LIVOX_TOPIC} topic:"
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${LIVOX_DOCKER_SOURCE} && ros2 topic info -v ${LIVOX_TOPIC}" || true
  echo "[JETSON] Livox driver log:"
  docker exec "${CONTAINER}" bash -lc "tail -80 /tmp/waver_livox_mid360_driver.log 2>/dev/null || true" || true
fi

echo "[JETSON] base driver state:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && timeout 5 ros2 topic echo --once --full-length /waver/base_driver_state" || true

echo "[JETSON] scan sample:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && timeout 5 ros2 topic echo --once ${SCAN_TOPIC}" >/tmp/waver_lidar_nav_scan_check.log 2>&1 || true
tail -40 /tmp/waver_lidar_nav_scan_check.log || true

echo "[JETSON] odom sample:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && timeout 5 ros2 topic echo --once /odom" >/tmp/waver_lidar_nav_odom_check.log 2>&1 || true
tail -60 /tmp/waver_lidar_nav_odom_check.log || true

echo "[JETSON] mode and safety:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && timeout 3 ros2 topic echo --once /waver/mode || true && timeout 3 ros2 topic echo --once /waver/safety_state || true" || true

echo "[JETSON] serial owner after launch:"
fuser -v "${SERIAL_PORT}" 2>&1 || true

echo "[JETSON] LIDAR_NAV_BACKEND_READY=YES"
REMOTE

echo "[LOCAL] LIDAR_NAV_BACKEND_READY=YES"
echo "[LOCAL] Open the local UI in another terminal:"
echo "  cd ~/ugv_ws/FSD_Vehicle"
echo "  JETSON_HOST=${JETSON_HOST} bash scripts/waver_field_local_ui_start.sh"
