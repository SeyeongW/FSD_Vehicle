#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${WAVER_BLACKBOX_DIR:-${ROOT}/reports/field_runs/${STAMP}}"
INCLUDE_HEAVY=false
PROFILE="${WAVER_BLACKBOX_PROFILE:-base}"
MODE="${WAVER_BLACKBOX_MODE:-unspecified}"
REMOTE=false
REMOTE_HOST=""
REMOTE_USER=""
REMOTE_CONTAINER=""
REMOTE_WORKSPACE=""

while [ "$#" -gt 0 ]; do
  case "$1" in
    --include-heavy-sensors) INCLUDE_HEAVY=true; shift ;;
    --profile) PROFILE="${2:?missing profile}"; shift 2 ;;
    --mode) MODE="${2:?missing mode}"; shift 2 ;;
    --remote) REMOTE=true; shift ;;
    --host) REMOTE_HOST="${2:?missing host}"; shift 2 ;;
    --user) REMOTE_USER="${2:?missing user}"; shift 2 ;;
    --container) REMOTE_CONTAINER="${2:?missing container}"; shift 2 ;;
    --workspace) REMOTE_WORKSPACE="${2:?missing workspace}"; shift 2 ;;
    --output-dir) OUT="${2:?missing output dir}"; shift 2 ;;
    -h|--help) echo "Usage: bash scripts/waver_blackbox_recorder.sh [--profile bird_mission] [--mode MODE] [--include-heavy-sensors] [--output-dir DIR]"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "${OUT}/rosbag"
cat >"${OUT}/metadata.yaml" <<EOF
generated_at: $(date --iso-8601=seconds)
include_heavy_sensors: ${INCLUDE_HEAVY}
profile: ${PROFILE}
mode: ${MODE}
remote_requested: ${REMOTE}
ros_domain_id: ${ROS_DOMAIN_ID:-}
source_manifest_hash: $(sha256sum "${ROOT}/reports/source_manifest.json" 2>/dev/null | awk '{print $1}' || true)
camera_lidar_calibration_hash: $(sha256sum "${ROOT}/config/sensors/camera_lidar_extrinsic.yaml" 2>/dev/null | awk '{print $1}' || true)
sound_backend: ${WAVER_SOUND_BACKEND:-disabled}
operator_ack_state:
  sound_hardware: ${WAVER_ACK_SOUND_HARDWARE:-0}
  local_sound_law: ${WAVER_ACK_LOCAL_SOUND_LAW:-0}
  operator_supervision: ${WAVER_ACK_OPERATOR_SUPERVISION:-0}
location_label: ${WAVER_FIELD_LOCATION_LABEL:-}
weather_notes: ${WAVER_FIELD_WEATHER_NOTES:-}
robot_config_hash: $(sha256sum "${ROOT}/config/real_profiles/bird_patrol_production.yaml" 2>/dev/null | awk '{print $1}' || true)
EOF

topics=(
  /cmd_vel
  /waver/cmd_vel_safety
  /waver/safety_state
  /waver/base_driver_state
  /waver/serial_owner_state
  /odom
  /odom_raw
  /imu/data_raw
  /scan
  /scan_safety
  /tf
  /tf_static
  /voltage
  /diagnostics
)
if [ "${PROFILE}" = "bird_mission" ]; then
  topics+=(
    /waver/lidar_objects
    /waver/elevated_dynamic_targets
    /waver/moving_target_valid
    /waver/moving_object_filter_state
    /waver/bird_detections_2d
    /waver/bird_detector_state
    /waver/bird_confirmed
    /waver/target_class
    /waver/target_confidence
    /waver/bird_fusion_state
    /waver/bird_fusion_sync_state
    /waver/bird_target_valid
    /waver/bird_target_pose_map
    /waver/object_mission_goal
    /waver/inspection_target_pose_map
    /waver/mission_state
    /waver/camera_alignment_state
    /waver/camera_target_centered
    /waver/sound_alert_state
    /waver/sound_task_active
    /waver/sound_task_done
    /waver/target_departed
    /waver/bird_mission_supervisor_state
  )
fi
if [ "${INCLUDE_HEAVY}" = "true" ]; then
  topics+=(/livox/lidar /livox/imu /camera/image_raw /camera/camera_info)
else
  topics+=(/camera/camera_info)
fi

echo "BLACKBOX_RECORDER_OUTPUT=${OUT}"
if [ "${REMOTE}" = "true" ]; then
  # shellcheck source=scripts/waver_field_env_load.sh
  source "${ROOT}/scripts/waver_field_env_load.sh"
  JETSON_HOST="${REMOTE_HOST:-${JETSON_HOST:-}}"
  JETSON_USER="${REMOTE_USER:-${JETSON_USER:-}}"
  CONTAINER="${REMOTE_CONTAINER:-${CONTAINER:-fsd_dev_jetson}}"
  REMOTE_WORKSPACE="${REMOTE_WORKSPACE:-/ros2_ws/ros2_ws5}"
  waver_field_env_require
  waver_field_env_ensure_password
  waver_ssh_cmd
  printf -v TOPIC_ARGS ' %q' "${topics[@]}"
  REMOTE_OUT="${WAVER_BLACKBOX_REMOTE_DIR:-${REMOTE_WORKSPACE}/reports/field_runs/${STAMP}}"
  exec "${WAVER_SSH_CMD[@]}" "${JETSON_USER}@${JETSON_HOST}" "docker exec -i ${CONTAINER} bash -lc 'cd ${REMOTE_WORKSPACE} && source /opt/ros/humble/setup.bash && source install_docker/setup.bash && mkdir -p ${REMOTE_OUT}/rosbag && exec ros2 bag record -o ${REMOTE_OUT}/rosbag/waver_blackbox${TOPIC_ARGS}'"
fi
exec ros2 bag record -o "${OUT}/rosbag/waver_blackbox" "${topics[@]}"
