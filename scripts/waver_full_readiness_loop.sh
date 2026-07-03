#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CYCLES=2
NO_REAL_HARDWARE=true
PAPER_STRICT=false
REPORT_ROOT="${WAVER_FULL_READINESS_REPORT_ROOT:-${ROOT}/reports/full_readiness_loop/$(date +%Y%m%d_%H%M%S)}"

usage() {
  cat <<'EOF'
Usage: bash scripts/waver_full_readiness_loop.sh [--cycles N] [--no-real-hardware] [--paper-strict]

Runs repeated local readiness checks. Real hardware motion is never commanded.
Gazebo/Docker/SSH checks record SKIP_WITH_REASON when the environment is absent.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --cycles) CYCLES="${2:?missing cycles}"; shift 2 ;;
    --no-real-hardware) NO_REAL_HARDWARE=true; shift ;;
    --paper-strict) PAPER_STRICT=true; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

mkdir -p "${REPORT_ROOT}"
summary="${REPORT_ROOT}/summary.csv"
echo "cycle,step,status,evidence" > "${summary}"

run_step() {
  local cycle="$1"
  local step="$2"
  shift 2
  local dir="${REPORT_ROOT}/cycle_${cycle}"
  mkdir -p "${dir}"
  local log="${dir}/${step}.log"
  local status="PASS"
  echo "[CYCLE ${cycle}] ${step}: $*"
  if ! (cd "${ROOT}" && "$@") >"${log}" 2>&1; then
    status="FAIL"
  fi
  if grep -q "SKIP_WITH_REASON" "${log}" && [ "${status}" = "PASS" ]; then
    status="SKIP_WITH_REASON"
  fi
  echo "${cycle},${step},${status},${log}" >> "${summary}"
  echo "FULL_READINESS_${step}_CYCLE_${cycle}=${status} evidence=${log}"
  if [ "${PAPER_STRICT}" = "true" ]; then
    [ "${status}" = "PASS" ]
  else
    [ "${status}" != "FAIL" ]
  fi
}

overall=0
skip_seen=0
for cycle in $(seq 1 "${CYCLES}"); do
  for spec in \
    "compileall python3 -m compileall -q scripts src/waver_patrol/waver_patrol src/waver_patrol/launch" \
    "no_ros_tests bash scripts/run_no_ros_unit_tests.sh" \
    "contract_check python3 scripts/waver_contract_check.py" \
    "clone_to_run bash scripts/waver_clone_to_run_acceptance.sh" \
    "remote_ui bash src/waver_patrol/scripts/waver_remote_ui_validation.sh --mock --cycles 1" \
    "docker_ssh_no_motion bash scripts/waver_field_docker_ssh_check.sh --local-compose --jetson-compose --no-motion --cycles 1" \
    "gazebo_keyboard bash src/waver_patrol/scripts/waver_gazebo_functional_validation_loop.sh --scenario keyboard_teleop_smoke --cycles 1"; do
    step="${spec%% *}"
    # shellcheck disable=SC2086
    run_step "${cycle}" ${spec} || overall=1
    last_status="$(tail -n 1 "${summary}" | awk -F, '{print $3}')"
    if [ "${last_status}" != "PASS" ] && [ "${last_status}" != "FAIL" ]; then
      skip_seen=1
    fi
  done
done

echo "FULL_READINESS_REPORT=${REPORT_ROOT}"
if [ "${overall}" -eq 0 ]; then
  if [ "${skip_seen}" -eq 1 ]; then
    echo "FULL_READINESS_LOOP=PASS_WITH_SKIPS"
  else
    echo "FULL_READINESS_LOOP=PASS"
  fi
else
  echo "FULL_READINESS_LOOP=FAIL"
fi
exit "${overall}"
