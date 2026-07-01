#!/usr/bin/env bash
set -euo pipefail

LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
# shellcheck source=scripts/waver_field_env_load.sh
source "${LOCAL_ROOT}/scripts/waver_field_env_load.sh"
waver_field_env_require
waver_field_env_ensure_password
if [ -n "${JETSON_PASS:-}" ] && [ "${WAVER_ALLOW_PASSWORD_SSH:-0}" != "1" ]; then
  echo "[SECURITY] Password SSH is disabled by default." >&2
  echo "[SECURITY] Set WAVER_ALLOW_PASSWORD_SSH=1 only for explicit field debugging." >&2
  echo "[SECURITY] Prefer SSH keys." >&2
  exit 1
fi
waver_ssh_cmd
waver_scp_cmd
SSH_CMD=("${WAVER_SSH_CMD[@]}")
SCP_CMD=("${WAVER_SCP_CMD[@]}")

JETSON_HOST_CANDIDATES="${JETSON_HOST_CANDIDATES:-${JETSON_HOST}}"
PATROL_ALLOW_OPEN_LOOP="${PATROL_ALLOW_OPEN_LOOP:-true}"
PATROL_WAYPOINT_FILE="${PATROL_WAYPOINT_FILE:-/ros2_ws/ros2_ws5/src/ugv_main/ugv_tools/waypoints/waver_0p2m_patrol.yaml}"
PATROL_STEP_DISTANCE_M="${PATROL_STEP_DISTANCE_M:-0.2}"
PATROL_FORWARD_DURATION_S="${PATROL_FORWARD_DURATION_S:-0.65}"
PATROL_TURN_DURATION_S="${PATROL_TURN_DURATION_S:-2.4}"
PATROL_FORWARD_SPEED="${PATROL_FORWARD_SPEED:-0.085}"
PATROL_TURN_SPEED="${PATROL_TURN_SPEED:-0.075}"
PATROL_TURN_WHEEL_RATIO="${PATROL_TURN_WHEEL_RATIO:-0.46}"
PATROL_TURN_MODE="${PATROL_TURN_MODE:-pivot}"

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
    if "${SSH_CMD[@]}" "${JETSON_USER}@${candidate}" "echo WAVER_JETSON_SSH_OK" >/tmp/waver_jetson_probe.log 2>&1; then
      JETSON_HOST="${candidate}"
      printf '%s\n' "${JETSON_HOST}" > "${HOME}/.waver_jetson_host"
      echo "[LOCAL] selected Jetson host: ${JETSON_HOST}"
      return 0
    fi
    tail -3 /tmp/waver_jetson_probe.log 2>/dev/null || true
  done
  echo "[LOCAL][ERROR] Could not reach Jetson SSH on any candidate: ${JETSON_HOST_CANDIDATES}" >&2
  echo "[LOCAL][ERROR] Local network:" >&2
  ip -4 addr show | sed 's/^/[LOCAL][NET] /' >&2 || true
  echo "[LOCAL][ERROR] Set JETSON_HOST=<current_jetson_ip> or reconnect PC/Jetson to the same hotspot." >&2
  exit 12
}

SYNC_FILES=(
  ".env"
  "docker-compose.jetson.yml"
  "docker/run.sh"
  "config/waver_field_env"
  "scripts/waver_field_env_load.sh"
  "src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py"
  "src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py"
  "src/ugv_main/ugv_tools/ugv_tools/waver_gazebo_patrol.py"
  "src/ugv_main/ugv_tools/waypoints/waver_0p3m_patrol.yaml"
  "src/ugv_main/ugv_tools/waypoints/waver_0p2m_patrol.yaml"
  "src/ugv_main/ugv_tools/waypoints/waver_0p1m_patrol.yaml"
)

if [ -f "${HOME}/.waver_jetson_host" ] && [ -z "${JETSON_HOST:-}" ]; then
  JETSON_HOST="$(head -n 1 "${HOME}/.waver_jetson_host")"
fi
select_jetson_host

if ! "${SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "test -f '${JETSON_WS}/docker/run.sh' && test -f '${JETSON_WS}/docker-compose.jetson.yml'" >/tmp/waver_backend_ws_probe.log 2>&1; then
  echo "[LOCAL] Jetson workspace is missing or incomplete; running bootstrap first"
  bash "${LOCAL_ROOT}/scripts/waver_field_bootstrap_jetson.sh"
fi

echo "[LOCAL] target Jetson: ${JETSON_USER}@${JETSON_HOST} ws=${JETSON_WS}"
echo "[LOCAL] syncing field-control source files to Jetson"
for rel in "${SYNC_FILES[@]}"; do
  [ -f "${LOCAL_ROOT}/${rel}" ] || continue
  remote_dir="${JETSON_WS}/$(dirname "${rel}")"
  "${SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "mkdir -p '${remote_dir}'"
  "${SCP_CMD[@]}" "${LOCAL_ROOT}/${rel}" "${JETSON_USER}@${JETSON_HOST}:${JETSON_WS}/${rel}" >/dev/null
done

"${SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "bash -s" -- \
  "${JETSON_WS}" "${CONTAINER}" "${SERIAL_PORT}" \
  "${FIELD_BUILD_IN_DOCKER}" "${PATROL_ALLOW_OPEN_LOOP}" "${PATROL_WAYPOINT_FILE}" \
  "${PATROL_FORWARD_DURATION_S}" "${PATROL_TURN_DURATION_S}" \
  "${PATROL_FORWARD_SPEED}" "${PATROL_TURN_SPEED}" "${PATROL_STEP_DISTANCE_M}" \
  "${PATROL_TURN_WHEEL_RATIO}" "${PATROL_TURN_MODE}" \
  "${SERIAL_PORT_BY_ID_PATTERN}" "${SERIAL_PORT_ALLOW_TTYUSB_FALLBACK}" "${SERIAL_PORT_ALLOW_TTYTHS_FALLBACK}" <<'REMOTE'
set -euo pipefail

JETSON_WS="$1"
CONTAINER="$2"
SERIAL_REQUEST="$3"
FIELD_BUILD_IN_DOCKER="$4"
PATROL_ALLOW_OPEN_LOOP="$5"
PATROL_WAYPOINT_FILE="$6"
PATROL_FORWARD_DURATION_S="$7"
PATROL_TURN_DURATION_S="$8"
PATROL_FORWARD_SPEED="$9"
PATROL_TURN_SPEED="${10}"
PATROL_STEP_DISTANCE_M="${11}"
PATROL_TURN_WHEEL_RATIO="${12}"
PATROL_TURN_MODE="${13}"
SERIAL_PORT_BY_ID_PATTERN="${14}"
SERIAL_PORT_ALLOW_TTYUSB_FALLBACK="${15}"
SERIAL_PORT_ALLOW_TTYTHS_FALLBACK="${16}"

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
    exit 19
  elif [ "${#ttyusb_candidates[@]}" -eq 1 ] && [ "${SERIAL_PORT_ALLOW_TTYUSB_FALLBACK}" = "true" ]; then
    SERIAL_PORT="${ttyusb_candidates[0]}"
    echo "[JETSON][WARN] falling back to ${SERIAL_PORT}" >&2
  elif [ "${#ttyusb_candidates[@]}" -gt 1 ]; then
    echo "[JETSON][ERROR] multiple /dev/ttyUSB* candidates; set SERIAL_PORT explicitly." >&2
    printf '  - %s\n' "${ttyusb_candidates[@]}" >&2
    exit 19
  elif [ "${SERIAL_PORT_ALLOW_TTYTHS_FALLBACK}" = "true" ] && [ -e /dev/ttyTHS1 ]; then
    SERIAL_PORT="/dev/ttyTHS1"
    echo "[JETSON][WARN] falling back to /dev/ttyTHS1" >&2
  else
    echo "[JETSON][ERROR] no Waver serial port found. Set SERIAL_PORT=/dev/serial/by-id/..." >&2
    exit 19
  fi
fi
echo "[JETSON] selected serial port: ${SERIAL_PORT}"
echo "[JETSON] patrol waypoint file: ${PATROL_WAYPOINT_FILE}"
echo "[JETSON] patrol open-loop fallback: ${PATROL_ALLOW_OPEN_LOOP}"
echo "[JETSON] field patrol calibration: step_distance=${PATROL_STEP_DISTANCE_M}m forward_duration=${PATROL_FORWARD_DURATION_S}s turn_duration=${PATROL_TURN_DURATION_S}s forward_speed=${PATROL_FORWARD_SPEED} turn_speed=${PATROL_TURN_SPEED} turn_wheel_ratio=${PATROL_TURN_WHEEL_RATIO} turn_mode=${PATROL_TURN_MODE}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON] starting Docker container via headless-safe runner"
  bash docker/run.sh jetson-up >/tmp/waver_docker_run.log 2>&1 || {
    echo "[JETSON][ERROR] docker/run.sh jetson-up failed. See /tmp/waver_docker_run.log"
    tail -80 /tmp/waver_docker_run.log || true
    exit 20
  }
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON][ERROR] Docker container ${CONTAINER} is not running"
  docker ps -a --format 'table {{.Names}}\t{{.Status}}'
  exit 21
fi

if [ "${FIELD_BUILD_IN_DOCKER}" = "auto" ] && ! docker exec "${CONTAINER}" bash -lc 'cd /ros2_ws/ros2_ws5 && test -f install_docker/setup.bash' >/dev/null 2>&1; then
  FIELD_BUILD_IN_DOCKER=true
fi

if [ "${FIELD_BUILD_IN_DOCKER}" = "true" ]; then
  echo "[JETSON] building jo packages inside Docker"
  docker exec "${CONTAINER}" bash -lc '
  set -eo pipefail
  cd /ros2_ws/ros2_ws5
  source /opt/ros/humble/setup.bash
  colcon --log-base log_docker build \
    --build-base build_docker \
    --install-base install_docker \
    --packages-select ugv_description ugv_tools waver_patrol
  '
else
  echo "[JETSON] FIELD_BUILD_IN_DOCKER=${FIELD_BUILD_IN_DOCKER}; using existing install_docker entry points"
fi

echo "[JETSON] overlaying field Python sources into install_docker"
docker exec "${CONTAINER}" bash -lc '
set -e
cd /ros2_ws/ros2_ws5
cp src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py \
  install_docker/waver_patrol/lib/python3.10/site-packages/waver_patrol/bridges/waver_base_driver_node.py
cp src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py \
  install_docker/waver_patrol/lib/python3.10/site-packages/waver_patrol/safety/safety_cmd_mux_node.py
cp src/ugv_main/ugv_tools/ugv_tools/waver_gazebo_patrol.py \
  install_docker/ugv_tools/lib/python3.10/site-packages/ugv_tools/waver_gazebo_patrol.py
'

echo "[JETSON] ensuring 0.2m waypoint file inside Docker workspace"
docker exec "${CONTAINER}" bash -lc "mkdir -p /ros2_ws/ros2_ws5/src/ugv_main/ugv_tools/waypoints && cat > /ros2_ws/ros2_ws5/src/ugv_main/ugv_tools/waypoints/waver_0p2m_patrol.yaml <<'YAML'
frame_id: odom
home:
  name: home
  x: 0.0
  y: 0.0
  yaw: 0.0
waypoints:
  - name: start
    x: 0.0
    y: 0.0
    yaw: 0.0
  - name: east_0p2
    x: 0.2
    y: 0.0
    yaw: 1.57
  - name: north_east_0p2
    x: 0.2
    y: 0.2
    yaw: 3.14
  - name: north_0p2
    x: 0.0
    y: 0.2
    yaw: -1.57
failure_policy:
  max_patrol_radius_m: 0.35
  recovery_attempts: 0
  human_estop_required: true
YAML"

echo "[JETSON] stopping stale ROS driver processes"
pkill -f "docker exec -i ${CONTAINER}.*WAVER_REMOTE_BRIDGE_TIMEOUT_S" 2>/dev/null || true
for pat in \
  waver_base_driver_node \
  safety_cmd_mux_node \
  mission_patrol_manager_node \
  waver_gazebo_patrol \
  serial_cmd_vel_bridge \
  waver_cmd_vel_serial_bridge; do
  pkill -f "${pat}" 2>/dev/null || true
  docker exec "${CONTAINER}" pkill -f "${pat}" 2>/dev/null || true
done
sleep 1

echo "[JETSON] serial owner before backend:"
fuser -v "${SERIAL_PORT}" 2>&1 || true

DOCKER_SOURCE='if [ -f /opt/ros/humble/install/setup.bash ]; then source /opt/ros/humble/install/setup.bash; fi; source /opt/ros/humble/setup.bash; if [ -f install_docker/setup.bash ]; then source install_docker/setup.bash; elif [ -f install/setup.bash ]; then source install/setup.bash; fi; export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"; export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"'

start_node() {
  local name="$1"
  shift
  echo "[JETSON] starting ${name}"
  docker exec -d "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && exec $* >/tmp/${name}.log 2>&1"
}

echo "[JETSON] starting Docker backend on ${SERIAL_PORT}"
start_node waver_mission_patrol_manager \
  ros2 run waver_patrol mission_patrol_manager_node --ros-args \
    -p use_nav2:=false \
    -p default_mode:=STANDBY \
    -p patrol_loop:=false \
    -p enable_sound_task:=false

start_node waver_safety_cmd_mux \
  ros2 run waver_patrol safety_cmd_mux_node --ros-args \
    -p mode_default:=STANDBY \
    -p nav2_cmd_topic:=/waver/cmd_vel_nav2_smooth \
    -p cmd_vel_auto_topic:=/waver/cmd_vel_nav2_smooth \
    -p manual_cmd_vel_topic:=/waver/manual_cmd_vel \
    -p require_scan:=false \
    -p ignore_scan_when_require_scan_false:=true \
    -p stop_on_adapter_degraded:=false \
    -p stop_on_battery_fault:=false \
    -p max_linear_speed:=0.12 \
    -p max_angular_speed:=0.16 \
    -p mapping_max_linear_speed:=0.12 \
    -p mapping_max_angular_speed:=0.16 \
    -p max_linear_delta_per_tick:=0.08 \
    -p max_angular_delta_per_tick:=0.12 \
    -p manual_override_timeout_sec:=0.12 \
    -p allow_manual_override_in_auto:=true \
    -p command_timeout_sec:=0.14 \
    -p timer_hz:=80.0

start_node waver_base_driver \
  ros2 run waver_patrol waver_base_driver_node --ros-args \
    -p serial_port:="${SERIAL_PORT}" \
    -p cmd_vel_topic:=/cmd_vel \
    -p command_protocol:=lr \
    -p command_rate_hz:=80.0 \
    -p cmd_timeout_s:=0.14 \
    -p stop_repeat:=10 \
    -p linear_gain:=2.5 \
    -p angular_gain:=0.45 \
    -p max_left_right:=0.48 \
    -p max_demo_speed:=0.48 \
    -p min_linear_ratio:=0.22 \
    -p wheel_delta_per_tick:=0.16 \
    -p pure_turn_mode:="${PATROL_TURN_MODE}" \
    -p pure_turn_min_ratio:="${PATROL_TURN_WHEEL_RATIO}" \
    -p pure_turn_max_ratio:="${PATROL_TURN_WHEEL_RATIO}" \
    -p mixed_turn_mode:=inside_brake \
    -p mixed_turn_inner_ratio:=0.0 \
    -p mixed_turn_outer_ratio:=0.22 \
    -p min_motor_voltage_v:=7.0 \
    -p feedback_request_enabled:=true \
    -p feedback_request_interval_s:=0.25 \
    -p publish_odom:=true \
    -p publish_tf:=true \
    -p odom_topic:=/odom \
    -p odom_frame:=odom \
    -p base_frame:=base_link \
    -p wheel_base_m:=0.28 \
    -p odom_distance_scale:=1.0

start_node waver_0p2_patrol \
  ros2 run ugv_tools waver_gazebo_patrol --ros-args \
    -p cmd_vel_topic:=/waver/cmd_vel_nav2_smooth \
    -p odom_topic:=/odom \
    -p enable_scan_assist:=false \
    -p lidar_required:=false \
    -p auto_start:=false \
    -p start_on_mission_command:=true \
    -p relative_waypoints_to_start:=true \
    -p reset_relative_origin_on_start:=true \
    -p waypoint_file:="${PATROL_WAYPOINT_FILE}" \
    -p allow_open_loop_without_odom:="${PATROL_ALLOW_OPEN_LOOP}" \
    -p open_loop_step_distance_m:="${PATROL_STEP_DISTANCE_M}" \
    -p open_loop_forward_duration_s:="${PATROL_FORWARD_DURATION_S}" \
    -p open_loop_sides_per_loop:=4 \
    -p open_loop_turn_duration_s:="${PATROL_TURN_DURATION_S}" \
    -p open_loop_turn_angular_speed:="${PATROL_TURN_SPEED}" \
    -p open_loop_turn_direction:=1.0 \
    -p open_loop_dwell_s:=0.25 \
    -p loop_count:=-1 \
    -p max_linear_speed:="${PATROL_FORWARD_SPEED}" \
    -p max_angular_speed:="${PATROL_TURN_SPEED}" \
    -p max_linear_accel:=0.22 \
    -p max_angular_accel:=0.18 \
    -p xy_tolerance:=0.04 \
    -p yaw_tolerance:=0.12 \
    -p approach_distance_m:=0.12 \
    -p heading_align_threshold_rad:=0.25 \
    -p stuck_timeout_s:=22.0 \
    -p max_recovery_attempts:=0

echo "[JETSON] waiting for ROS backend nodes"
required_nodes=(
  /mission_patrol_manager_node
  /safety_cmd_mux_node
  /waver_base_driver_node
  /waver_gazebo_patrol
)

nodes=""
for _ in $(seq 1 12); do
  nodes="$(docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && ros2 node list 2>/dev/null" || true)"
  missing=0
  for node in "${required_nodes[@]}"; do
    if ! grep -qx "${node}" <<<"${nodes}"; then
      missing=1
      break
    fi
  done
  [ "${missing}" -eq 0 ] && break
  sleep 1
done

echo "[JETSON] ROS nodes:"
echo "${nodes}" | sort

missing_nodes=()
for node in "${required_nodes[@]}"; do
  if ! grep -qx "${node}" <<<"${nodes}"; then
    missing_nodes+=("${node}")
  fi
done

if [ "${#missing_nodes[@]}" -ne 0 ]; then
  echo "[JETSON][ERROR] backend did not start required nodes: ${missing_nodes[*]}"
  echo "[JETSON][ERROR] node logs:"
  for log in /tmp/waver_mission_patrol_manager.log /tmp/waver_safety_cmd_mux.log /tmp/waver_base_driver.log /tmp/waver_0p2_patrol.log; do
    echo "----- ${log} -----"
    docker exec "${CONTAINER}" bash -lc "tail -120 ${log} 2>/dev/null || true"
  done
  exit 30
fi

echo "[JETSON] /cmd_vel chain:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && ros2 topic info -v /cmd_vel"

echo "[JETSON] /waver/manual_cmd_vel:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && ros2 topic info -v /waver/manual_cmd_vel || true"

echo "[JETSON] base driver state:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 4 ros2 topic echo --once --full-length /waver/base_driver_state || true"

echo "[JETSON] odom sample:"
if docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 4 ros2 topic echo --once /odom" >/tmp/waver_backend_odom_check.log 2>&1; then
  cat /tmp/waver_backend_odom_check.log
  echo "[JETSON] ODOM_READY=YES"
else
  cat /tmp/waver_backend_odom_check.log || true
  if [ "${PATROL_ALLOW_OPEN_LOOP}" = "true" ]; then
    echo "[JETSON][WARN] ODOM_READY=NO. WAVE ROVER field mode will use supervised calibrated open-loop ${PATROL_STEP_DISTANCE_M}m micro-patrol."
  else
    echo "[JETSON][WARN] ODOM_READY=NO. True 0.2m waypoint patrol will wait; do not call open-loop movement a waypoint PASS."
  fi
fi

echo "[JETSON] safety state:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 4 ros2 topic echo --once /waver/safety_state || true"

echo "[JETSON] mode:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ros2_ws5 && ${DOCKER_SOURCE} && timeout 4 ros2 topic echo --once /waver/mode || true"

echo "[JETSON] serial owner after backend:"
fuser -v "${SERIAL_PORT}" 2>&1 || true

echo "[JETSON] BACKEND_READY=YES"
echo "[JETSON] Now run local UI in another local PC terminal:"
echo "  cd ~/ros2_ws5/FSD_Vehicle"
echo "  bash scripts/waver_field_local_ui_start.sh"
REMOTE
