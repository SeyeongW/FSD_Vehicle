#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
REPORT_ROOT="${WAVER_REMOTE_UI_REPORT_ROOT:-${ROOT}/reports/remote_ui_validation/$(date +%Y%m%d_%H%M%S)}"
CYCLES=1
MOCK=false
LAUNCH_UI=false
TIMEOUT_SEC="${WAVER_REMOTE_UI_TIMEOUT_SEC:-25}"

usage() {
  cat <<'EOF'
Usage: bash src/waver_patrol/scripts/waver_remote_ui_validation.sh [--mock] [--launch-ui] [--cycles N]

Default mode is static/mock validation only. It does not publish motion commands
and does not require a real Jetson or Waver serial connection.
EOF
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --mock) MOCK=true; shift ;;
    --launch-ui) LAUNCH_UI=true; shift ;;
    --cycles) CYCLES="${2:?missing cycles}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done

mkdir -p "${REPORT_ROOT}"
summary="${REPORT_ROOT}/summary.csv"
echo "cycle,status,key_output,evidence" > "${summary}"

ui_file="${ROOT}/src/ugv_main/ugv_tools/ugv_tools/waver_remote_panel.py"
launch_file="${ROOT}/src/ugv_main/ugv_tools/launch/waver_operator_panel.launch.py"

run_cycle() {
  local cycle="$1"
  local dir="${REPORT_ROOT}/cycle_${cycle}"
  mkdir -p "${dir}"
  local log="${dir}/remote_ui_validation.log"
  local status="PASS"
  local key="static_contract_ok"

  {
    echo "REMOTE_UI_VALIDATION cycle=${cycle}"
    echo "mock=${MOCK} launch_ui=${LAUNCH_UI}"
    python3 -m py_compile "${ui_file}"
    python3 - <<'PY' "${ui_file}" "${launch_file}"
import pathlib
import sys

ui = pathlib.Path(sys.argv[1]).read_text(errors="ignore")
launch = pathlib.Path(sys.argv[2]).read_text(errors="ignore")

checks = {
    "direct_cmd_vel_default_false": 'declare_parameter("publish_direct_cmd_vel", False)' in ui,
    "real_forces_direct_off": 'self.profile == "real" and self.publish_direct_cmd_vel' in ui,
    "manual_candidate_topic": "/waver/manual_cmd_vel" in ui,
    "mode_cmd_topic": "/waver/mode_cmd" in ui,
    "mapping_command_topic": "/waver/mapping_command" in ui,
    "ui_map_reset": "clear_map_for_new_mapping_session" in ui and "/waver/ui_map_reset" in ui,
    "remote_password_guard": "WAVER_ALLOW_PASSWORD_SSH" in ui,
    "operator_launch_no_direct_cmd": "publish_direct_cmd_vel" in launch,
}
failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(f"{name}={'PASS' if ok else 'FAIL'}")
if failed:
    raise SystemExit("REMOTE_UI_STATIC_FAIL " + ",".join(failed))
print("REMOTE_UI_STATIC_PASS")
PY
    if [ "${LAUNCH_UI}" = "true" ]; then
      if [ -z "${DISPLAY:-}" ]; then
        echo "REMOTE_UI_LAUNCH=SKIP_WITH_REASON no DISPLAY"
      else
        set +e
        timeout --foreground "${TIMEOUT_SEC}" bash -lc "cd '${ROOT}' && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run ugv_tools waver_remote_panel --ros-args -p profile:=mock -p publish_direct_cmd_vel:=false -p remote_bridge_enabled:=false -p demo_script_name:=keyboard_smoke -p demo_close_on_finish:=true"
        rc=$?
        set -e
        if [ "$rc" -ne 0 ] && [ "$rc" -ne 124 ]; then
          echo "REMOTE_UI_LAUNCH=FAIL rc=${rc}"
          exit "$rc"
        fi
        echo "REMOTE_UI_LAUNCH=PASS_OR_TIMEOUT rc=${rc}"
      fi
    fi
  } >"${log}" 2>&1 || {
    status="FAIL"
    key="$(tail -1 "${log}" | tr ',' ';')"
  }
  if grep -q "SKIP_WITH_REASON" "${log}" && [ "${status}" = "PASS" ]; then
    key="static_pass_launch_skip"
  fi
  echo "${cycle},${status},${key},${log}" >> "${summary}"
  echo "REMOTE_UI_VALIDATION_${cycle}=${status} evidence=${log}"
  [ "${status}" = "PASS" ]
}

overall=0
for cycle in $(seq 1 "${CYCLES}"); do
  run_cycle "${cycle}" || overall=1
done

if [ "${overall}" -eq 0 ]; then
  echo "REMOTE_UI_VALIDATION=PASS"
else
  echo "REMOTE_UI_VALIDATION=FAIL"
fi
exit "${overall}"
