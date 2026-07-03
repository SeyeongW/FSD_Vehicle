#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="${WAVER_BLACKBOX_DIR:-${ROOT}/reports/field_runs/${STAMP}}"
INCLUDE_HEAVY=false

while [ "$#" -gt 0 ]; do
  case "$1" in
    --include-heavy-sensors) INCLUDE_HEAVY=true; shift ;;
    --output-dir) OUT="${2:?missing output dir}"; shift 2 ;;
    -h|--help) echo "Usage: bash scripts/waver_blackbox_recorder.sh [--include-heavy-sensors] [--output-dir DIR]"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

mkdir -p "${OUT}/rosbag"
cat >"${OUT}/metadata.yaml" <<EOF
generated_at: $(date --iso-8601=seconds)
include_heavy_sensors: ${INCLUDE_HEAVY}
ros_domain_id: ${ROS_DOMAIN_ID:-}
EOF

topics=(/cmd_vel /waver/cmd_vel_safety /waver/safety_state /waver/base_driver_state /waver/serial_owner_state /odom /odom_raw /imu/data_raw /scan /scan_safety /tf /tf_static /voltage /diagnostics)
if [ "${INCLUDE_HEAVY}" = "true" ]; then
  topics+=(/livox/lidar /livox/imu /camera/image_raw /camera/camera_info)
fi

echo "BLACKBOX_RECORDER_OUTPUT=${OUT}"
exec ros2 bag record -o "${OUT}/rosbag/waver_blackbox" "${topics[@]}"
