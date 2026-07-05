#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${WAVER_BIRD_PATROL_PROFILE:-}"
PROFILE_EXPLICIT=false
MODE="sensor-live"
MAP_PATH=""
SERIAL_PORT="${SERIAL_PORT:-}"
BIRD_MODEL="${BIRD_MODEL_PATH:-}"
CAMERA_EXTRINSIC="${ROOT}/config/sensors/camera_lidar_extrinsic.yaml"
ENABLE_SOUND_OUTPUT=false
START_BLACKBOX=false
SCAN_TOPIC="${SCAN_TOPIC:-}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-}"
CAMERA_IMAGE_TOPIC="${CAMERA_IMAGE_TOPIC:-}"
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-}"
MAX_LINEAR_SPEED="${SAFETY_MAX_LINEAR_SPEED:-}"
MAX_ANGULAR_SPEED="${SAFETY_MAX_ANGULAR_SPEED:-}"
SPEED_TIER="${WAVER_SPEED_TIER:-}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --profile) PROFILE="${2:?missing profile}"; PROFILE_EXPLICIT=true; shift 2 ;;
    --mode) MODE="${2:?missing mode}"; shift 2 ;;
    --map) MAP_PATH="${2:?missing map}"; shift 2 ;;
    --serial-port) SERIAL_PORT="${2:?missing serial port}"; shift 2 ;;
    --bird-model) BIRD_MODEL="${2:?missing model path}"; shift 2 ;;
    --camera-extrinsic) CAMERA_EXTRINSIC="${2:?missing extrinsic path}"; shift 2 ;;
    --scan-topic) SCAN_TOPIC="${2:?missing scan topic}"; shift 2 ;;
    --pointcloud-topic) POINTCLOUD_TOPIC="${2:?missing pointcloud topic}"; shift 2 ;;
    --camera-image-topic) CAMERA_IMAGE_TOPIC="${2:?missing image topic}"; shift 2 ;;
    --camera-info-topic) CAMERA_INFO_TOPIC="${2:?missing camera info topic}"; shift 2 ;;
    --max-linear-speed) MAX_LINEAR_SPEED="${2:?missing max linear speed}"; shift 2 ;;
    --max-angular-speed) MAX_ANGULAR_SPEED="${2:?missing max angular speed}"; shift 2 ;;
    --speed-tier) SPEED_TIER="${2:?missing speed tier}"; shift 2 ;;
    --enable-sound-output) ENABLE_SOUND_OUTPUT=true; shift ;;
    --blackbox) START_BLACKBOX=true; shift ;;
    -h|--help)
      echo "Usage: bash scripts/waver_bird_patrol_field_start.sh --mode source|sensor-live|lidar-tracking|detector-live|fusion-live|inspection-dry-run|supervised-deterrence|supervised-bird-patrol|autonomous-patrol [--speed-tier first-wheel-on|supervised-low-speed|production] [--enable-sound-output]"
      exit 0
      ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [ -z "${PROFILE}" ]; then
  case "${MODE}" in
    source)
      PROFILE="bird_patrol_production"
      ;;
    sensor-live)
      PROFILE="sensor_live"
      ;;
    lidar-tracking)
      PROFILE="lidar_nav_backend"
      ;;
    detector-live|fusion-live|inspection-dry-run)
      PROFILE="inspection_dry_run"
      ;;
    supervised-deterrence|supervised-bird-patrol)
      PROFILE="supervised_bird_patrol"
      ;;
    autonomous-patrol)
      PROFILE="autonomous_bird_patrol_locked"
      ;;
    *)
      echo "BIRD_PATROL_READY=FAIL"
      echo "[BIRD_PATROL][ERROR] unsupported mode: ${MODE}" >&2
      exit 2
      ;;
  esac
fi

READINESS_MODE="${MODE}"
if [ "${READINESS_MODE}" = "supervised-bird-patrol" ]; then
  READINESS_MODE="supervised-deterrence"
fi

DEFAULT_FIELD_READINESS_LEVEL="L5"
case "${MODE}" in
  source) DEFAULT_FIELD_READINESS_LEVEL="L0" ;;
  sensor-live|lidar-tracking|detector-live|fusion-live) DEFAULT_FIELD_READINESS_LEVEL="L2" ;;
  inspection-dry-run) DEFAULT_FIELD_READINESS_LEVEL="L3" ;;
  supervised-deterrence|supervised-bird-patrol) DEFAULT_FIELD_READINESS_LEVEL="L4" ;;
  autonomous-patrol) DEFAULT_FIELD_READINESS_LEVEL="L5" ;;
esac

if [ -z "${SPEED_TIER}" ]; then
  case "${MODE}" in
    source|sensor-live|lidar-tracking|detector-live|fusion-live) SPEED_TIER="monitoring-only" ;;
    inspection-dry-run) SPEED_TIER="first-wheel-on" ;;
    supervised-deterrence|supervised-bird-patrol) SPEED_TIER="supervised-low-speed" ;;
    autonomous-patrol) SPEED_TIER="production" ;;
  esac
fi

case "${SPEED_TIER}" in
  monitoring-only)
    : "${MAX_LINEAR_SPEED:=0.0}"
    : "${MAX_ANGULAR_SPEED:=0.0}"
    ;;
  first-wheel-on)
    : "${MAX_LINEAR_SPEED:=0.05}"
    : "${MAX_ANGULAR_SPEED:=0.20}"
    ;;
  supervised-low-speed)
    : "${MAX_LINEAR_SPEED:=0.08}"
    : "${MAX_ANGULAR_SPEED:=0.25}"
    ;;
  production)
    if [ "${WAVER_ACK_PRODUCTION_SPEED_EVIDENCE:-0}" != "1" ]; then
      echo "BIRD_PATROL_READY=FAIL"
      echo "[BIRD_PATROL][ERROR] production speed tier requires WAVER_ACK_PRODUCTION_SPEED_EVIDENCE=1" >&2
      exit 5
    fi
    ;;
  *)
    echo "BIRD_PATROL_READY=FAIL"
    echo "[BIRD_PATROL][ERROR] unsupported speed tier: ${SPEED_TIER}" >&2
    exit 2
    ;;
esac

python3 - "$SPEED_TIER" "$MAX_LINEAR_SPEED" "$MAX_ANGULAR_SPEED" <<'PY'
import sys
tier, lin, ang = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
limits = {
    "monitoring-only": (0.0, 0.0),
    "first-wheel-on": (0.05, 0.20),
    "supervised-low-speed": (0.12, 0.35),
}
if tier in limits:
    max_lin, max_ang = limits[tier]
    if lin > max_lin + 1e-9 or ang > max_ang + 1e-9:
        raise SystemExit(f"speed tier {tier} exceeded: linear={lin} angular={ang}")
if tier == "supervised-low-speed" and (lin < 0.08 - 1e-9 or ang < 0.25 - 1e-9):
    raise SystemExit(f"supervised-low-speed defaults must be at least 0.08/0.25: linear={lin} angular={ang}")
PY

PROFILE_PATH="${ROOT}/config/real_profiles/${PROFILE}.yaml"
if [ ! -f "${PROFILE_PATH}" ]; then
  echo "BIRD_PATROL_READY=FAIL"
  echo "[BIRD_PATROL][ERROR] profile not found: ${PROFILE_PATH}" >&2
  exit 3
fi

if [ "${ENABLE_SOUND_OUTPUT}" = "true" ]; then
  for ack in WAVER_ACK_SOUND_HARDWARE WAVER_ACK_LOCAL_SOUND_LAW WAVER_ACK_OPERATOR_SUPERVISION; do
    if [ "${!ack:-}" != "1" ]; then
      echo "BIRD_PATROL_READY=FAIL"
      echo "[BIRD_PATROL][ERROR] --enable-sound-output requires ${ack}=1" >&2
      exit 4
    fi
  done
else
  export ENABLE_SOUND_STACK=true
fi

if [ "${START_BLACKBOX}" = "true" ]; then
  bash "${ROOT}/scripts/waver_blackbox_recorder.sh" --profile bird_mission --output-dir "${ROOT}/reports/field_runs/bird_$(date +%Y%m%d_%H%M%S)" &
  BLACKBOX_PID=$!
  trap 'kill ${BLACKBOX_PID} 2>/dev/null || true' EXIT
fi

if [ "${MODE}" = "source" ]; then
  if python3 "${ROOT}/scripts/waver_bird_mission_readiness_check.py" --mode source --profile "${PROFILE_PATH}" --strict --no-hardware; then
    echo "BIRD_PATROL_READY=PASS"
    exit 0
  fi
  echo "BIRD_PATROL_READY=FAIL"
  exit 1
fi

export WAVER_REAL_PROFILE="${PROFILE}"
export WAVER_REAL_PROFILE_PATH="${PROFILE_PATH}"
export WAVER_PRODUCT_LAUNCH="bird_patrol_production.launch.py"
export FIELD_READINESS_LEVEL="${FIELD_READINESS_LEVEL:-${DEFAULT_FIELD_READINESS_LEVEL}}"
export FIELD_READINESS_STRICT="${FIELD_READINESS_STRICT:-true}"
export WAVER_SPEED_TIER="${SPEED_TIER}"
export BIRD_MODEL_PATH="${BIRD_MODEL}"
export CAMERA_LIDAR_EXTRINSIC="${CAMERA_EXTRINSIC}"
export SERIAL_PORT="${SERIAL_PORT}"
[ -n "${MAP_PATH}" ] && export MAP_PATH
[ -n "${SCAN_TOPIC}" ] && export SCAN_TOPIC
[ -n "${POINTCLOUD_TOPIC}" ] && export POINTCLOUD_TOPIC
[ -n "${CAMERA_IMAGE_TOPIC}" ] && export CAMERA_IMAGE_TOPIC
[ -n "${CAMERA_INFO_TOPIC}" ] && export CAMERA_INFO_TOPIC
[ -n "${MAX_LINEAR_SPEED}" ] && export SAFETY_MAX_LINEAR_SPEED="${MAX_LINEAR_SPEED}"
[ -n "${MAX_ANGULAR_SPEED}" ] && export SAFETY_MAX_ANGULAR_SPEED="${MAX_ANGULAR_SPEED}"
echo "[BIRD_PATROL] mode=${MODE} readiness_mode=${READINESS_MODE} profile=${PROFILE} explicit_profile=${PROFILE_EXPLICIT} readiness=${FIELD_READINESS_LEVEL} speed_tier=${SPEED_TIER}"
if [ "${ENABLE_SOUND_OUTPUT}" = "true" ]; then
  export ENABLE_SOUND_OUTPUT=true
else
  export ENABLE_SOUND_OUTPUT=false
fi

set +e
bash "${ROOT}/scripts/waver_field_lidar_nav_backend_start.sh"
backend_rc=$?
python3 "${ROOT}/scripts/waver_bird_mission_readiness_check.py" --mode "${READINESS_MODE}" --profile "${PROFILE_PATH}" --strict
ready_rc=$?
set -e

if [ "${backend_rc}" -eq 0 ] && [ "${ready_rc}" -eq 0 ]; then
  echo "BIRD_PATROL_READY=PASS"
  exit 0
fi
if [ "${backend_rc}" -eq 0 ] && [ "${MODE}" = "sensor-live" ] && [ "${BIRD_PATROL_ALLOW_DEGRADED:-0}" = "1" ]; then
  echo "BIRD_PATROL_READY=PASS_DEGRADED"
  exit 0
fi
echo "BIRD_PATROL_READY=FAIL"
exit 1
