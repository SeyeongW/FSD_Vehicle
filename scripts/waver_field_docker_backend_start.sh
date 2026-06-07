#!/usr/bin/env bash
set -euo pipefail

JETSON_HOST="${JETSON_HOST:-10.63.240.150}"
JETSON_USER="${JETSON_USER:-sw}"
JETSON_PASS="${JETSON_PASS:-12341234}"
JETSON_WS="${JETSON_WS:-/home/sw/ros2_ws2/FSD_Vehicle}"
CONTAINER="${CONTAINER:-fsd_dev_jetson}"
SERIAL_PORT="${SERIAL_PORT:-auto}"
FIELD_BUILD_IN_DOCKER="${FIELD_BUILD_IN_DOCKER:-false}"
PATROL_ALLOW_OPEN_LOOP="${PATROL_ALLOW_OPEN_LOOP:-true}"
PATROL_WAYPOINT_FILE="${PATROL_WAYPOINT_FILE:-/ros2_ws/ugv_ws/src/ugv_main/ugv_tools/waypoints/waver_0p2m_patrol.yaml}"
PATROL_STEP_DISTANCE_M="${PATROL_STEP_DISTANCE_M:-0.2}"
PATROL_FORWARD_DURATION_S="${PATROL_FORWARD_DURATION_S:-0.65}"
PATROL_TURN_DURATION_S="${PATROL_TURN_DURATION_S:-2.1}"
PATROL_FORWARD_SPEED="${PATROL_FORWARD_SPEED:-0.06}"
PATROL_TURN_SPEED="${PATROL_TURN_SPEED:-0.025}"
PATROL_TURN_WHEEL_RATIO="${PATROL_TURN_WHEEL_RATIO:-0.27}"
PATROL_TURN_MODE="${PATROL_TURN_MODE:-pivot}"

SSH_OPTS=(-o StrictHostKeyChecking=no -o ConnectTimeout=8)
if command -v sshpass >/dev/null 2>&1; then
  SSH_CMD=(sshpass -p "${JETSON_PASS}" ssh "${SSH_OPTS[@]}")
  SCP_CMD=(sshpass -p "${JETSON_PASS}" scp "${SSH_OPTS[@]}")
else
  echo "[LOCAL] sshpass not found; falling back to normal ssh." >&2
  echo "[LOCAL] Enter Jetson password when prompted: ${JETSON_PASS}" >&2
  SSH_CMD=(ssh "${SSH_OPTS[@]}")
  SCP_CMD=(scp "${SSH_OPTS[@]}")
fi

LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SYNC_FILES=(
  "src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py"
  "src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py"
  "src/ugv_main/ugv_tools/ugv_tools/waver_gazebo_patrol.py"
  "src/ugv_main/ugv_tools/waypoints/waver_0p3m_patrol.yaml"
  "src/ugv_main/ugv_tools/waypoints/waver_0p2m_patrol.yaml"
  "src/ugv_main/ugv_tools/waypoints/waver_0p1m_patrol.yaml"
)

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
  "${PATROL_TURN_WHEEL_RATIO}" "${PATROL_TURN_MODE}" <<'REMOTE'
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

cd "${JETSON_WS}"
echo "[JETSON] repo=$(pwd) branch=$(git branch --show-current 2>/dev/null || echo unknown) head=$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"

SERIAL_PORT="${SERIAL_REQUEST}"
if [ "${SERIAL_PORT}" = "auto" ]; then
  SERIAL_PORT="$(ls /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_* 2>/dev/null | head -n 1 || true)"
  if [ -z "${SERIAL_PORT}" ]; then
    SERIAL_PORT="$(ls /dev/ttyUSB* 2>/dev/null | head -n 1 || true)"
  fi
  if [ -z "${SERIAL_PORT}" ]; then
    SERIAL_PORT="/dev/ttyTHS1"
  fi
fi
echo "[JETSON] selected serial port: ${SERIAL_PORT}"
echo "[JETSON] patrol waypoint file: ${PATROL_WAYPOINT_FILE}"
echo "[JETSON] patrol open-loop fallback: ${PATROL_ALLOW_OPEN_LOOP}"
echo "[JETSON] field patrol calibration: step_distance=${PATROL_STEP_DISTANCE_M}m forward_duration=${PATROL_FORWARD_DURATION_S}s turn_duration=${PATROL_TURN_DURATION_S}s forward_speed=${PATROL_FORWARD_SPEED} turn_speed=${PATROL_TURN_SPEED} turn_wheel_ratio=${PATROL_TURN_WHEEL_RATIO} turn_mode=${PATROL_TURN_MODE}"

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON] starting Docker container via main_s-style runner"
  bash docker/run.sh jetson >/tmp/waver_docker_run.log 2>&1 || {
    echo "[JETSON][ERROR] docker/run.sh jetson failed. See /tmp/waver_docker_run.log"
    tail -80 /tmp/waver_docker_run.log || true
    exit 20
  }
fi

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[JETSON][ERROR] Docker container ${CONTAINER} is not running"
  docker ps -a --format 'table {{.Names}}\t{{.Status}}'
  exit 21
fi

if [ "${FIELD_BUILD_IN_DOCKER}" = "true" ]; then
  echo "[JETSON] building jo packages inside Docker"
  docker exec "${CONTAINER}" bash -lc '
  set -eo pipefail
  cd /ros2_ws/ugv_ws
  source /opt/ros/humble/install/setup.bash
  source /opt/ros/humble/setup.bash
  colcon --log-base log_docker build \
    --build-base build_docker \
    --install-base install_docker \
    --packages-select ugv_description ugv_tools waver_patrol
  '
else
  echo "[JETSON] FIELD_BUILD_IN_DOCKER=false; using existing install_docker entry points"
fi

echo "[JETSON] overlaying field Python sources into install_docker"
docker exec "${CONTAINER}" bash -lc '
set -e
cd /ros2_ws/ugv_ws
cp src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py \
  install_docker/waver_patrol/lib/python3.10/site-packages/waver_patrol/bridges/waver_base_driver_node.py
cp src/waver_patrol/waver_patrol/safety/safety_cmd_mux_node.py \
  install_docker/waver_patrol/lib/python3.10/site-packages/waver_patrol/safety/safety_cmd_mux_node.py
cp src/ugv_main/ugv_tools/ugv_tools/waver_gazebo_patrol.py \
  install_docker/ugv_tools/lib/python3.10/site-packages/ugv_tools/waver_gazebo_patrol.py
'

echo "[JETSON] ensuring 0.2m waypoint file inside Docker workspace"
docker exec "${CONTAINER}" bash -lc "mkdir -p /ros2_ws/ugv_ws/src/ugv_main/ugv_tools/waypoints && cat > /ros2_ws/ugv_ws/src/ugv_main/ugv_tools/waypoints/waver_0p2m_patrol.yaml <<'YAML'
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

DOCKER_SOURCE='source /opt/ros/humble/install/setup.bash && source /opt/ros/humble/setup.bash && source install_docker/setup.bash && export ROS_DOMAIN_ID=30 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp'

start_node() {
  local name="$1"
  shift
  echo "[JETSON] starting ${name}"
  docker exec -d "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && exec $* >/tmp/${name}.log 2>&1"
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
    -p max_linear_speed:=0.08 \
    -p max_angular_speed:=0.08 \
    -p mapping_max_linear_speed:=0.08 \
    -p mapping_max_angular_speed:=0.08 \
    -p max_linear_delta_per_tick:=0.04 \
    -p max_angular_delta_per_tick:=0.035 \
    -p manual_override_timeout_sec:=0.30 \
    -p allow_manual_override_in_auto:=true \
    -p command_timeout_sec:=0.35 \
    -p timer_hz:=60.0

start_node waver_base_driver \
  ros2 run waver_patrol waver_base_driver_node --ros-args \
    -p serial_port:="${SERIAL_PORT}" \
    -p cmd_vel_topic:=/cmd_vel \
    -p command_protocol:=lr \
    -p command_rate_hz:=60.0 \
    -p cmd_timeout_s:=0.35 \
    -p stop_repeat:=10 \
    -p linear_gain:=2.5 \
    -p angular_gain:=0.35 \
    -p max_left_right:=0.38 \
    -p max_demo_speed:=0.38 \
    -p min_linear_ratio:=0.18 \
    -p wheel_delta_per_tick:=0.06 \
    -p pure_turn_mode:="${PATROL_TURN_MODE}" \
    -p pure_turn_min_ratio:="${PATROL_TURN_WHEEL_RATIO}" \
    -p pure_turn_max_ratio:="${PATROL_TURN_WHEEL_RATIO}" \
    -p mixed_turn_mode:=inside_brake \
    -p mixed_turn_inner_ratio:=0.0 \
    -p mixed_turn_outer_ratio:=0.12 \
    -p min_motor_voltage_v:=7.0 \
    -p feedback_request_enabled:=true \
    -p feedback_request_interval_s:=0.3 \
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
    -p max_linear_accel:=0.12 \
    -p max_angular_accel:=0.04 \
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
  nodes="$(docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && ros2 node list 2>/dev/null" || true)"
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
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && ros2 topic info -v /cmd_vel"

echo "[JETSON] /waver/manual_cmd_vel:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && ros2 topic info -v /waver/manual_cmd_vel || true"

echo "[JETSON] base driver state:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && timeout 4 ros2 topic echo --once --full-length /waver/base_driver_state || true"

echo "[JETSON] odom sample:"
if docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && timeout 4 ros2 topic echo --once /odom" >/tmp/waver_backend_odom_check.log 2>&1; then
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
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && timeout 4 ros2 topic echo --once /waver/safety_state || true"

echo "[JETSON] mode:"
docker exec "${CONTAINER}" bash -lc "cd /ros2_ws/ugv_ws && ${DOCKER_SOURCE} && timeout 4 ros2 topic echo --once /waver/mode || true"

echo "[JETSON] serial owner after backend:"
fuser -v "${SERIAL_PORT}" 2>&1 || true

echo "[JETSON] BACKEND_READY=YES"
echo "[JETSON] Now run local UI in another local PC terminal:"
echo "  cd ~/ros2_ws2/FSD_Vehicle"
echo "  bash scripts/waver_field_local_ui_start.sh"
REMOTE
