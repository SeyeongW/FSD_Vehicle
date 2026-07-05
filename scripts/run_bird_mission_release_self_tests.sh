#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

export WAVER_RELEASE_SELF_TEST=1
export WAVER_NO_HARDWARE=1
export WAVER_BLOCK_SERIAL=1
export WAVER_DISABLE_SOUND_OUTPUT=1
export PYTHONPATH="${ROOT}/src/waver_patrol:${ROOT}/src/ugv_main/ugv_tools:${PYTHONPATH:-}"

REPORT_DIR="${ROOT}/reports/release_self_test"
mkdir -p "${REPORT_DIR}"
REPORT="${REPORT_DIR}/latest_bird_mission_release_self_test.json"
STEP_DIR="${REPORT_DIR}/steps"
mkdir -p "${STEP_DIR}"
STEPS_JSONL="${REPORT_DIR}/latest_bird_mission_release_self_test_steps.jsonl"
: > "${STEPS_JSONL}"
START_TS="$(date -Iseconds)"
STATUS="PASS"
FINDINGS=()

record_step() {
  local label="$1"
  local timeout_sec="$2"
  local rc="$3"
  local status="$4"
  local log_path="$5"
  shift 5
  python3 - "${STEPS_JSONL}" "${label}" "${timeout_sec}" "${rc}" "${status}" "${log_path}" "$*" <<'PY'
import json
import sys
from pathlib import Path

jsonl = Path(sys.argv[1])
label = sys.argv[2]
timeout_sec = float(sys.argv[3])
returncode = int(sys.argv[4])
status = sys.argv[5]
log_path = Path(sys.argv[6])
command = sys.argv[7]
try:
    tail = log_path.read_text(errors="replace")[-4000:]
except FileNotFoundError:
    tail = ""
with jsonl.open("a", encoding="utf-8") as f:
    f.write(json.dumps({
        "label": label,
        "timeout_sec": timeout_sec,
        "returncode": returncode,
        "status": status,
        "command": command,
        "log": str(log_path),
        "output_tail": tail,
    }, ensure_ascii=False, sort_keys=True) + "\n")
PY
}

run_step() {
  local label="$1"
  local timeout_sec="$2"
  shift 2
  local log_path="${STEP_DIR}/${label}.log"
  echo "[RELEASE_SELF_TEST] ${label} timeout=${timeout_sec}s: $*"
  set +e
  timeout --kill-after=5s "${timeout_sec}s" "$@" >"${log_path}" 2>&1
  local rc=$?
  set -e
  local step_status="PASS"
  if [ "${rc}" -eq 124 ] || [ "${rc}" -eq 137 ]; then
    step_status="TIMEOUT"
    STATUS="FAIL"
    FINDINGS+=("${label}:timeout:${timeout_sec}s")
  elif [ "${rc}" -ne 0 ]; then
    step_status="FAIL"
    STATUS="FAIL"
    FINDINGS+=("${label}:exit:${rc}")
  fi
  record_step "${label}" "${timeout_sec}" "${rc}" "${step_status}" "${log_path}" "$@"
  tail -80 "${log_path}" || true
}

run_step compileall "${RELEASE_SELF_TEST_TIMEOUT_COMPILEALL:-90}" python3 -m compileall -q scripts src/waver_patrol/waver_patrol src/waver_patrol/launch src/waver_patrol/scripts
run_step shell_syntax "${RELEASE_SELF_TEST_TIMEOUT_SHELL_SYNTAX:-30}" bash -lc 'find scripts -maxdepth 1 -type f -name "*.sh" -print0 | xargs -0 -r bash -n'
run_step no_ros_unit_tests "${RELEASE_SELF_TEST_TIMEOUT_NO_ROS_UNIT_TESTS:-240}" bash scripts/run_no_ros_unit_tests.sh
run_step contract "${RELEASE_SELF_TEST_TIMEOUT_CONTRACT:-90}" python3 scripts/waver_contract_check.py --allow-source-archive-without-git
run_step source_readiness "${RELEASE_SELF_TEST_TIMEOUT_SOURCE_READINESS:-90}" python3 scripts/waver_bird_mission_readiness_check.py --mode source --strict --no-hardware
run_step launch_contract "${RELEASE_SELF_TEST_TIMEOUT_LAUNCH_CONTRACT:-90}" python3 scripts/waver_launch_contract_check.py --profile config/real_profiles/bird_patrol_production.yaml
run_step bridge_regression "${RELEASE_SELF_TEST_TIMEOUT_BRIDGE_REGRESSION:-90}" python3 scripts/waver_field_bridge_regression_check.py --dry-run
run_step command_chain "${RELEASE_SELF_TEST_TIMEOUT_COMMAND_CHAIN:-90}" python3 scripts/waver_command_chain_check.py --dry-run --profile config/real_profiles/bird_patrol_production.yaml

PYTEST_TARGETS=(
  src/waver_patrol/test/test_bird_mission_not_removed.py
  src/waver_patrol/test/test_bird_patrol_production_profile.py
  src/waver_patrol/test/test_bird_entrypoint_uses_production_launch.py
  src/waver_patrol/test/test_bird_detector_deployment_contract.py
  src/waver_patrol/test/test_camera_lidar_fusion_gate.py
  src/waver_patrol/test/test_camera_alignment_production_contract.py
  src/waver_patrol/test/test_sound_deterrent_real_backend_gate.py
  src/waver_patrol/test/test_bird_mission_policy.py
  src/waver_patrol/test/test_bird_ready_degraded_policy.py
  src/waver_patrol/test/test_launch_contract_required_args.py
  src/waver_patrol/test/test_field_bridge_regression_contract.py
  src/waver_patrol/test/test_command_chain_collision_monitor_contract.py
  src/waver_patrol/test/test_remote_ui_slam_bird_contract.py
  src/waver_patrol/test/test_ui_slam_bird_detection_smoke_contract.py
  src/waver_patrol/test/test_local_operator_station_contract.py
  src/waver_patrol/test/test_repository_cleanup_policy.py
  src/waver_patrol/test/test_readiness_truthfulness_contract.py
)

EXISTING_TARGETS=()
for target in "${PYTEST_TARGETS[@]}"; do
  if [ -f "${target}" ]; then
    EXISTING_TARGETS+=("${target}")
  fi
done
if [ "${#EXISTING_TARGETS[@]}" -gt 0 ]; then
  run_step targeted_pytest "${RELEASE_SELF_TEST_TIMEOUT_TARGETED_PYTEST:-240}" python3 -m pytest -q "${EXISTING_TARGETS[@]}"
fi

python3 - "${REPORT}" "${STEPS_JSONL}" "${STATUS}" "${START_TS}" "${FINDINGS[@]}" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
steps_jsonl = Path(sys.argv[2])
status = sys.argv[3]
start_ts = sys.argv[4]
findings = sys.argv[5:]
steps = []
if steps_jsonl.exists():
    steps = [json.loads(line) for line in steps_jsonl.read_text().splitlines() if line.strip()]
path.write_text(json.dumps({
    "status": status,
    "started_at": start_ts,
    "finished_at": __import__("datetime").datetime.now().astimezone().isoformat(),
    "findings": findings,
    "steps": steps,
    "release_safe_subset": True,
}, indent=2, sort_keys=True) + "\n")
PY

echo "BIRD_MISSION_RELEASE_SELF_TEST=${STATUS}"
echo "BIRD_MISSION_RELEASE_SELF_TEST_REPORT=${REPORT}"
test "${STATUS}" = "PASS"
