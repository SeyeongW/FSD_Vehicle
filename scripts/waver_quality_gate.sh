#!/usr/bin/env bash
set -u
set -o pipefail

WORKSPACE_ROOT=""
REPORT_DIR=""
NO_HARDWARE=false
RUN_COLCON_TEST=true
RUN_BUILD=true
RUN_LEGACY_STYLE_TESTS=false
BUILD_PACKAGES="ugv_description ugv_gazebo ugv_tools waver_patrol waver_seo_tracking waver_experiment_logger"
TEST_PACKAGES="waver_patrol waver_experiment_logger"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --root)
      WORKSPACE_ROOT="$2"; shift 2 ;;
    --report-dir)
      REPORT_DIR="$2"; shift 2 ;;
    --no-hardware)
      NO_HARDWARE=true; shift ;;
    --skip-colcon-test)
      RUN_COLCON_TEST=false; shift ;;
    --skip-build)
      RUN_BUILD=false; shift ;;
    --include-legacy-style-tests)
      RUN_LEGACY_STYLE_TESTS=true; shift ;;
    *)
      echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [ -z "$WORKSPACE_ROOT" ]; then
  if [ -f "src/waver_patrol/package.xml" ]; then
    WORKSPACE_ROOT="$PWD"
  else
    WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  fi
fi

cd "$WORKSPACE_ROOT" || exit 2
STAMP="$(date +%Y%m%d_%H%M%S)"
if [ -z "$REPORT_DIR" ]; then
  REPORT_DIR="reports/quality_gate/$STAMP"
fi
mkdir -p "$REPORT_DIR"

exec > >(tee "$REPORT_DIR/quality_gate.log") 2>&1

echo "WAVER_QUALITY_GATE_START=$STAMP"
echo "WORKSPACE_ROOT=$WORKSPACE_ROOT"
echo "NO_HARDWARE=$NO_HARDWARE"
echo "BUILD_PACKAGES=$BUILD_PACKAGES"
echo "TEST_PACKAGES=$TEST_PACKAGES"
echo "RUN_LEGACY_STYLE_TESTS=$RUN_LEGACY_STYLE_TESTS"

if [ "$NO_HARDWARE" != "true" ]; then
  echo "ERROR: this quality gate must be run with --no-hardware for unattended use." >&2
  exit 2
fi

export WAVER_ALLOW_HARDWARE="${WAVER_ALLOW_HARDWARE:-0}"
export WAVER_NO_HARDWARE="${WAVER_NO_HARDWARE:-1}"
export WAVER_BLOCK_SERIAL="${WAVER_BLOCK_SERIAL:-1}"
export WAVER_DISABLE_SOUND_OUTPUT="${WAVER_DISABLE_SOUND_OUTPUT:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

if [ -e /dev/serial/by-id ]; then
  echo "SERIAL_DEVICES_PRESENT=YES"
  echo "NOTE: quality gate will not open serial devices."
else
  echo "SERIAL_DEVICES_PRESENT=NO"
fi

git status --short > "$REPORT_DIR/git_status_short.txt" 2>&1 || true
git branch --show-current > "$REPORT_DIR/git_branch.txt" 2>&1 || true

set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi
set -u

FAILURES=0

run_step() {
  local name="$1"; shift
  echo "===== STEP $name ====="
  "$@" > "$REPORT_DIR/${name}.log" 2>&1
  local status=$?
  cat "$REPORT_DIR/${name}.log"
  echo "STEP_${name}_STATUS=$status"
  if [ "$status" -ne 0 ]; then
    FAILURES=$((FAILURES + 1))
  fi
}

run_step "hardware_guard" bash scripts/waver_hardware_guard.sh --check-env

run_step "python_compileall_waver_patrol" python3 -m compileall -q src/waver_patrol
if [ -d src/waver_experiment_logger ]; then
  run_step "python_compileall_waver_experiment_logger" python3 -m compileall -q src/waver_experiment_logger
fi
if [ -d src/ugv_main/ugv_tools ]; then
  run_step "python_compileall_ugv_tools" python3 -m compileall -q src/ugv_main/ugv_tools
fi
if [ -d src/ugv_main/ugv_gazebo ]; then
  run_step "python_compileall_ugv_gazebo" python3 -m compileall -q src/ugv_main/ugv_gazebo
fi

run_step "contract_check" python3 scripts/waver_contract_check.py --root "$WORKSPACE_ROOT" --report-dir "$REPORT_DIR"

run_step "launch_dryrun_check" bash scripts/waver_launch_dryrun_check.sh --root "$WORKSPACE_ROOT" --report-dir "$REPORT_DIR"
run_step "rosbag_replay_check_optional" bash scripts/waver_rosbag_replay_check.sh --root "$WORKSPACE_ROOT" --report-dir "$REPORT_DIR" --optional

if [ "$RUN_BUILD" = "true" ]; then
  run_step "colcon_build_selected" colcon build --symlink-install --packages-select $BUILD_PACKAGES
  set +u
  if [ -f install/setup.bash ]; then
    source install/setup.bash
  fi
  set -u
fi

run_step "launch_import_check" python3 - <<'PY'
import importlib.util
import pathlib
import traceback

# Keep the unattended quality gate focused on Waver's deployable surface.
# Optional third-party/example packages under src/ugv_else can require package
# installation side effects before import and should not fail the real-vehicle
# safety contract by default.
launch_roots = [
    pathlib.Path("src/waver_patrol/launch"),
    pathlib.Path("src/ugv_main/ugv_gazebo/launch"),
    pathlib.Path("src/ugv_main/ugv_tools/launch"),
]
failures = 0
for root in launch_roots:
    if not root.exists():
        continue
    for path in sorted(root.glob("**/*.launch.py")):
        if any(part in {"build", "install", "log"} for part in path.parts):
            continue
        spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)  # type: ignore[union-attr]
            if hasattr(module, "generate_launch_description"):
                print("LAUNCH_IMPORT_OK", path)
        except Exception:
            failures += 1
            print("LAUNCH_IMPORT_FAIL", path)
            traceback.print_exc()
if failures:
    raise SystemExit(failures)
PY

if [ "$RUN_COLCON_TEST" = "true" ]; then
  if [ "$RUN_LEGACY_STYLE_TESTS" = "true" ]; then
    TEST_PACKAGES="ugv_tools $TEST_PACKAGES"
  fi
  run_step "colcon_test_selected" colcon test --packages-select $TEST_PACKAGES
  run_step "real_vehicle_contract_pytest" python3 -m pytest -q src/waver_patrol/test/test_real_vehicle_contract_skeleton.py
  for pkg in $TEST_PACKAGES; do
    if [ -d "build/$pkg" ]; then
      run_step "colcon_test_result_$pkg" colcon test-result --verbose --test-result-base "build/$pkg"
    fi
  done
fi

python3 - "$REPORT_DIR" "$FAILURES" <<'PY'
import json
import pathlib
import sys

report_dir = pathlib.Path(sys.argv[1])
failures = int(sys.argv[2])
contract = report_dir / "contract_report.json"
contract_score = 0
contract_result = "UNKNOWN"
if contract.exists():
    data = json.loads(contract.read_text())
    contract_score = int(data.get("score", 0))
    contract_result = str(data.get("result", "UNKNOWN"))
score = contract_score + failures * 1000
result = "PASS" if failures == 0 and contract_result == "PASS" else "FAIL"
summary = {
    "result": result,
    "score": score,
    "step_failures": failures,
    "contract_result": contract_result,
    "contract_score": contract_score,
}
(report_dir / "quality_gate_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
(report_dir / "quality_gate_summary.txt").write_text(
    "\n".join(f"{k}={v}" for k, v in summary.items()) + "\n"
)
latest_dir = pathlib.Path("reports/quality_gate")
latest_dir.mkdir(parents=True, exist_ok=True)
(latest_dir / "latest.json").write_text(json.dumps(summary | {"report_dir": str(report_dir)}, indent=2) + "\n")
latest_markdown = "\n".join(
    [
        "# Waver Quality Gate Latest",
        "",
        f"- result: {result}",
        f"- score: {score}",
        f"- report_dir: {report_dir}",
        f"- step_failures: {failures}",
        f"- contract_result: {contract_result}",
        f"- contract_score: {contract_score}",
        "",
        "Next suggested fix:",
        "- See contract_report.txt and failed step logs in the report directory.",
    ]
) + "\n"
(latest_dir / "latest.md").write_text(latest_markdown)
(latest_dir / "latest_summary.md").write_text(latest_markdown)
print(f"WAVER_QUALITY_GATE={result} SCORE={score} STEP_FAILURES={failures} CONTRACT={contract_result}")
raise SystemExit(0 if result == "PASS" else 1)
PY
