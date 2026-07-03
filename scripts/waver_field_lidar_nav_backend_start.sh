#!/usr/bin/env bash
set -euo pipefail

# Real LiDAR/Nav2 backend starter.
# This preserves the validated local-PC -> SSH -> Jetson -> Docker -> Waver USB
# field topology, but starts the real localization/Nav2 mission stack instead of
# the supervised open-loop micro-patrol helper.

LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${LOCAL_ROOT}/scripts/waver_field_env_load.sh"
waver_field_env_require
waver_field_env_ensure_password
waver_ssh_cmd
waver_scp_cmd
SSH_CMD=("${WAVER_SSH_CMD[@]}")
SCP_CMD=("${WAVER_SCP_CMD[@]}")
JETSON_HOST_CANDIDATES="${JETSON_HOST_CANDIDATES:-${JETSON_HOST}}"

MAP_PATH="${MAP_PATH:-/ros2_ws/ros2_ws5/maps/waver_latest_map.yaml}"
WAYPOINT_FILE="${WAYPOINT_FILE:-/ros2_ws/ros2_ws5/src/waver_patrol/waypoints/waver_real_0p5m_square_patrol.yaml}"
MISSION_PARAMS_FILE="${MISSION_PARAMS_FILE:-/ros2_ws/ros2_ws5/src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-/ros2_ws/ros2_ws5/src/waver_patrol/config/nav2_params_waver_real.yaml}"
ODOM_SOURCE="${ODOM_SOURCE:-ekf}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-${LIVOX_POINTCLOUD_TOPIC:-/livox/lidar}}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
START_LIVOX_DRIVER="${START_LIVOX_DRIVER:-true}"
LIVOX_TOPIC="${LIVOX_TOPIC:-${POINTCLOUD_TOPIC}}"
LIVOX_CONFIG_PATH="${LIVOX_CONFIG_PATH:-/ros2_ws/ros2_ws5/install_docker/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json}"
LIVOX_FRAME_ID="${LIVOX_FRAME_ID:-livox}"
LIVOX_HOST_IP="${LIVOX_HOST_IP:-192.168.1.50}"
LIVOX_SENSOR_IP="${LIVOX_SENSOR_IP:-192.168.1.102}"
LIVOX_PUBLISH_FREQ="${LIVOX_PUBLISH_FREQ:-10.0}"
LIVOX_BD_CODE="${LIVOX_BD_CODE:-livox0000000001}"
BIRD_MODEL_PATH="${BIRD_MODEL_PATH:-${bird_model_path:-}}"
CAMERA_IMAGE_TOPIC="${CAMERA_IMAGE_TOPIC:-/camera/image_raw}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/camera_info}"
SAFETY_MAX_LINEAR_SPEED="${SAFETY_MAX_LINEAR_SPEED:-0.05}"
SAFETY_MAX_ANGULAR_SPEED="${SAFETY_MAX_ANGULAR_SPEED:-0.20}"
ENABLE_BIRD_STACK="${ENABLE_BIRD_STACK:-false}"
ENABLE_SOUND_STACK="${ENABLE_SOUND_STACK:-false}"
ENABLE_WAVER_BASE_DRIVER="${ENABLE_WAVER_BASE_DRIVER:-true}"
REQUIRE_SCAN="${REQUIRE_SCAN:-true}"
WAVER_FIELD_MODE="${WAVER_FIELD_MODE:-production}"
FIELD_READINESS_LEVEL="${FIELD_READINESS_LEVEL:-L3}"
FIELD_READINESS_STRICT="${FIELD_READINESS_STRICT:-true}"
SERIAL_PORT_BY_ID_PATTERN="${SERIAL_PORT_BY_ID_PATTERN:-/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_*}"
SERIAL_PORT_ALLOW_TTYUSB_FALLBACK="${SERIAL_PORT_ALLOW_TTYUSB_FALLBACK:-false}"
SERIAL_PORT_ALLOW_TTYTHS_FALLBACK="${SERIAL_PORT_ALLOW_TTYTHS_FALLBACK:-false}"

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
  "scripts"
  "docs"
  "config/real_profiles"
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
  "${LIVOX_FRAME_ID}" "${LIVOX_PUBLISH_FREQ}" "${LIVOX_BD_CODE}" \
  "${LIVOX_HOST_IP}" "${LIVOX_SENSOR_IP}" "${BIRD_MODEL_PATH}" \
  "${CAMERA_IMAGE_TOPIC}" "${CAMERA_INFO_TOPIC}" \
  "${SERIAL_PORT_BY_ID_PATTERN}" "${SERIAL_PORT_ALLOW_TTYUSB_FALLBACK}" "${SERIAL_PORT_ALLOW_TTYTHS_FALLBACK}" \
  "${ENABLE_WAVER_BASE_DRIVER}" "${REQUIRE_SCAN}" "${WAVER_FIELD_MODE}" "${FIELD_READINESS_LEVEL}" "${FIELD_READINESS_STRICT}" <<'REMOTE'
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
LIVOX_HOST_IP="${22}"
LIVOX_SENSOR_IP="${23}"
BIRD_MODEL_PATH="${24}"
CAMERA_IMAGE_TOPIC="${25}"
CAMERA_INFO_TOPIC="${26}"
SERIAL_PORT_BY_ID_PATTERN="${27}"
SERIAL_PORT_ALLOW_TTYUSB_FALLBACK="${28}"
SERIAL_PORT_ALLOW_TTYTHS_FALLBACK="${29}"
ENABLE_WAVER_BASE_DRIVER="${30}"
REQUIRE_SCAN="${31}"
WAVER_FIELD_MODE="${32}"
FIELD_READINESS_LEVEL="${33}"
FIELD_READINESS_STRICT="${34}"
BIRD_MODEL_LAUNCH_ARG=""
if [ -n "${BIRD_MODEL_PATH}" ]; then
  BIRD_MODEL_LAUNCH_ARG="bird_model_path:=${BIRD_MODEL_PATH}"
fi

cd "${JETSON_WS}"
echo "[JETSON] repo=$(pwd) branch=$(git branch --show-current 2>/dev/null || echo unknown) head=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"

SERIAL_PORT="${SERIAL_REQUEST}"
if [ "${SERIAL_PORT}" = "auto" ]; then
  shopt -s nullglob
  by_id_candidates=(${SERIAL_PORT_BY_ID_PATTERN})
  ttyusb_candidates=(/dev/ttyUSB*)
  shopt -u nullglob
  if [ "${#by_id_candidates[@]}" -eq 1 ]; then
    SERIAL_PORT="${by_id_candidates[0]}"
  elif [ "${#by_id_candidates[@]}" -gt 1 ]; then
    echo "[JETSON][ERROR] multiple Waver by-id serial candidates:" >&2
    printf '  - %s\n' "${by_id_candidates[@]}" >&2
    exit 20
  elif [ "${#ttyusb_candidates[@]}" -eq 1 ] && [ "${SERIAL_PORT_ALLOW_TTYUSB_FALLBACK}" = "true" ]; then
    SERIAL_PORT="${ttyusb_candidates[0]}"
    echo "[JETSON][WARN] falling back to ${SERIAL_PORT}" >&2
  elif [ "${#ttyusb_candidates[@]}" -gt 1 ]; then
    echo "[JETSON][ERROR] multiple /dev/ttyUSB* candidates; set SERIAL_PORT explicitly." >&2
    printf '  - %s\n' "${ttyusb_candidates[@]}" >&2
    exit 20
  elif [ "${SERIAL_PORT_ALLOW_TTYTHS_FALLBACK}" = "true" ] && [ -e /dev/ttyTHS1 ]; then
    SERIAL_PORT="/dev/ttyTHS1"
    echo "[JETSON][WARN] falling back to /dev/ttyTHS1" >&2
  else
    echo "[JETSON][ERROR] no Waver by-id serial port found. Connect Waver USB or set SERIAL_PORT=/dev/serial/by-id/..." >&2
    exit 20
  fi
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON] starting Docker container via docker/run.sh jetson"
  bash docker/run.sh jetson-up >/tmp/waver_lidar_nav_docker_run.log 2>&1 || {
    echo "[JETSON][ERROR] docker/run.sh jetson-up failed. See /tmp/waver_lidar_nav_docker_run.log"
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
  docker exec "${CONTAINER}" mkdir -p /ros2_ws/ros2_ws5/maps
  docker cp "${JETSON_WS}/maps/." "${CONTAINER}:/ros2_ws/ros2_ws5/maps/" >/dev/null 2>&1 || true
fi

if [ "${FIELD_BUILD_IN_DOCKER}" = "auto" ] && ! docker exec "${CONTAINER}" bash -lc 'cd /ros2_ws/ros2_ws5 && test -f install_docker/setup.bash' >/dev/null 2>&1; then
  FIELD_BUILD_IN_DOCKER=true
fi

if [ "${FIELD_BUILD_IN_DOCKER}" = "true" ]; then
  echo "[JETSON] building selected packages inside Docker"
  docker exec "${CONTAINER}" bash -lc '
  set -eo pipefail
  cd /ros2_ws/ros2_ws5
  source /opt/ros/humble/setup.bash
  colcon --log-base log_docker build \
    --build-base build_docker \
    --install-base install_docker \
    --packages-select ugv_description ugv_nav ugv_tools waver_patrol
  '
fi

if [ "${WAVER_FIELD_MODE}" = "dev" ]; then
  echo "[JETSON][DEV] overlaying source launch/config/python into install_docker"
  docker exec "${CONTAINER}" bash -lc '
set -e
cd /ros2_ws/ros2_ws5
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
else
  echo "[JETSON] production mode: hot overlay disabled; using built install_docker artifacts"
  docker exec "${CONTAINER}" bash -lc 'cd /ros2_ws/ros2_ws5 && test -f install_docker/setup.bash && source /opt/ros/humble/setup.bash && source install_docker/setup.bash && ros2 pkg prefix waver_patrol >/dev/null && ros2 pkg executables waver_patrol >/dev/null' || {
    echo "[JETSON][ERROR] production mode requires built install_docker artifacts. Set FIELD_BUILD_IN_DOCKER=true or WAVER_FIELD_MODE=dev for development overlay." >&2
    exit 31
  }
fi

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

DOCKER_SOURCE='if [ -f /opt/ros/humble/install/setup.bash ]; then source /opt/ros/humble/install/setup.bash; fi; source /opt/ros/humble/setup.bash; if [ -f install_docker/setup.bash ]; then source install_docker/setup.bash; elif [ -f install/setup.bash ]; then source install/setup.bash; fi; export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"; export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"'
LIVOX_DOCKER_SOURCE="${DOCKER_SOURCE}"
ENABLE_RL="false"
if [ "${ODOM_SOURCE}" = "ekf" ]; then
  ENABLE_RL="true"
fi

if [ "${START_LIVOX_DRIVER}" = "true" ]; then
  echo "[JETSON] checking livox_ros_driver2 inside Docker"
  if ! docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${LIVOX_DOCKER_SOURCE} && ros2 pkg prefix livox_ros_driver2 >/dev/null"; then
    echo "[JETSON][ERROR] livox_ros_driver2 is not built in Docker install_docker." >&2
    echo "[JETSON][ERROR] Run: bash scripts/waver_setup_livox_mid360_docker.sh" >&2
    exit 30
  fi
  echo "[JETSON] patching Livox Mid-360 config host_ip=${LIVOX_HOST_IP} lidar_ip=${LIVOX_SENSOR_IP}"
  docker exec \
    -e LIVOX_CONFIG_PATH="${LIVOX_CONFIG_PATH}" \
    -e LIVOX_HOST_IP="${LIVOX_HOST_IP}" \
    -e LIVOX_SENSOR_IP="${LIVOX_SENSOR_IP}" \
    "${CONTAINER}" python3 - <<'PY'
import json
import os
from pathlib import Path

path = Path(os.environ["LIVOX_CONFIG_PATH"])
source = Path("/ros2_ws/ros2_ws5/src/livox_ros_driver2/config/MID360_config.json")
if not path.exists() and source.exists():
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(source.read_text())
if not path.exists():
    raise SystemExit(f"Livox config not found: {path}")

data = json.loads(path.read_text())
host_ip = os.environ.get("LIVOX_HOST_IP", "").strip()
sensor_ip = os.environ.get("LIVOX_SENSOR_IP", "").strip()
if host_ip:
    host = data.setdefault("MID360", {}).setdefault("host_net_info", {})
    for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
        host[key] = host_ip
if sensor_ip:
    configs = data.setdefault("lidar_configs", [])
    if not configs:
        configs.append({})
    configs[0]["ip"] = sensor_ip
path.write_text(json.dumps(data, indent=2) + "\n")
print(f"LIVOX_CONFIG_PATCHED path={path} host_ip={host_ip} lidar_ip={sensor_ip}")
PY
  echo "[JETSON] starting Livox Mid-360 driver: livox/lidar -> ${LIVOX_TOPIC}"
  docker exec -d "${CONTAINER}" bash -lc "
  cd /ros2_ws/ros2_ws5
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
if [ "${ENABLE_BIRD_STACK}" = "true" ] && [ -z "${BIRD_MODEL_PATH}" ]; then
  echo "[JETSON][WARN] ENABLE_BIRD_STACK=true but BIRD_MODEL_PATH is empty; bird_confirmed will remain false until a model/detector bridge is provided."
fi

docker exec -d "${CONTAINER}" bash -lc "
cd /ros2_ws/ros2_ws5
${DOCKER_SOURCE}
exec ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  use_sim_time:=false \
  real_profile:=true \
  use_nav2:=true \
  default_mode:=STANDBY \
  enable_waver_base_driver:=${ENABLE_WAVER_BASE_DRIVER} \
  serial_port:=${SERIAL_PORT} \
  start_serial_bridge:=false \
  start_base_feedback:=false \
  include_existing_ugv_driver:=false \
  odom_source:=${ODOM_SOURCE} \
  enable_robot_localization:=${ENABLE_RL} \
  enable_velocity_smoother:=true \
  require_scan:=${REQUIRE_SCAN} \
  scan_source:=mid360 \
  scan_source_safety:=mid360 \
  scan_source_slam:=mid360 \
  pointcloud_topic:=${POINTCLOUD_TOPIC} \
  camera_image_topic:=${CAMERA_IMAGE_TOPIC} \
  camera_info_topic:=${CAMERA_INFO_TOPIC} \
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
  ${BIRD_MODEL_LAUNCH_ARG} \
  > /tmp/waver_lidar_nav_backend.log 2>&1
"

echo "[JETSON] waiting for backend graph"
for _ in $(seq 1 25); do
  nodes="$(docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && ros2 node list 2>/dev/null" || true)"
  if grep -q "/waver_base_driver_node" <<<"${nodes}" && grep -q "/safety_cmd_mux_node" <<<"${nodes}" && grep -q "/mission_patrol_manager_node" <<<"${nodes}"; then
    break
  fi
  sleep 1
done

echo "[JETSON] ROS nodes:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && ros2 node list 2>/dev/null | sort" || true

echo "[JETSON] /cmd_vel chain:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && ros2 topic info -v /cmd_vel" || true

if [ "${START_LIVOX_DRIVER}" = "true" ]; then
  echo "[JETSON] Livox Mid-360 package:"
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${LIVOX_DOCKER_SOURCE} && ros2 pkg prefix livox_ros_driver2" || true
  echo "[JETSON] ${LIVOX_TOPIC} topic:"
  docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${LIVOX_DOCKER_SOURCE} && ros2 topic info -v ${LIVOX_TOPIC}" || true
  echo "[JETSON] Livox driver log:"
  docker exec "${CONTAINER}" bash -lc "tail -80 /tmp/waver_livox_mid360_driver.log 2>/dev/null || true" || true
fi

echo "[JETSON] base driver state:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 5 ros2 topic echo --once --full-length /waver/base_driver_state" || true

echo "[JETSON] scan sample:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 5 ros2 topic echo --once ${SCAN_TOPIC}" >/tmp/waver_lidar_nav_scan_check.log 2>&1 || true
tail -40 /tmp/waver_lidar_nav_scan_check.log || true

echo "[JETSON] odom sample:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 5 ros2 topic echo --once /odom" >/tmp/waver_lidar_nav_odom_check.log 2>&1 || true
tail -60 /tmp/waver_lidar_nav_odom_check.log || true

echo "[JETSON] mode and safety:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 3 ros2 topic echo --once /waver/mode || true && timeout 3 ros2 topic echo --once /waver/safety_state || true" || true

echo "[JETSON] serial owner after launch:"
fuser -v "${SERIAL_PORT}" 2>&1 || true

READINESS_ARGS=(--level "${FIELD_READINESS_LEVEL}" --scan-topic "${SCAN_TOPIC}" --odom-source "${ODOM_SOURCE}" --require-scan "${REQUIRE_SCAN}" --enable-waver-base-driver "${ENABLE_WAVER_BASE_DRIVER}" --enable-bird-stack "${ENABLE_BIRD_STACK}" --enable-sound-output "false" --serial-port "${SERIAL_PORT}")
if [ "${FIELD_READINESS_STRICT}" = "true" ]; then
  READINESS_ARGS+=(--strict)
fi
echo "[JETSON] running field readiness checker: ${READINESS_ARGS[*]}"
if docker exec \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  -e SERIAL_PORT="${SERIAL_PORT}" \
  -e ODOM_SOURCE="${ODOM_SOURCE}" \
  -e REQUIRE_SCAN="${REQUIRE_SCAN}" \
  -e ENABLE_WAVER_BASE_DRIVER="${ENABLE_WAVER_BASE_DRIVER}" \
  -e ENABLE_BIRD_STACK="${ENABLE_BIRD_STACK}" \
  -e ENABLE_SOUND_OUTPUT="false" \
  "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && python3 scripts/waver_field_readiness_check.py ${READINESS_ARGS[*]}" | tee /tmp/waver_lidar_nav_readiness.log; then
  readiness_status="$(awk -F= '/^FIELD_READINESS=/{print $2}' /tmp/waver_lidar_nav_readiness.log | tail -1)"
else
  readiness_status="FAIL"
fi
case "${readiness_status}" in
  PASS)
    echo "[JETSON] LIDAR_NAV_BACKEND_READY=PASS"
    ;;
  PASS_LIMITED)
    echo "[JETSON] LIDAR_NAV_BACKEND_READY=PASS_LIMITED"
    ;;
  *)
    echo "[JETSON] LIDAR_NAV_BACKEND_READY=FAIL"
    exit 40
    ;;
esac
REMOTE

echo "[LOCAL] LIDAR_NAV_BACKEND_STARTED=YES"
echo "[LOCAL] Backend final readiness is printed by Jetson as LIDAR_NAV_BACKEND_READY=PASS|PASS_LIMITED|FAIL"
echo "[LOCAL] Open the local UI in another terminal:"
echo "  cd ~/ros2_ws5/FSD_Vehicle"
echo "  JETSON_HOST=${JETSON_HOST} bash scripts/waver_field_local_ui_start.sh"
