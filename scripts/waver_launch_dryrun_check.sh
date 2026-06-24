#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
REPORT_DIR="${REPORT_DIR:-}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --root)
      WORKSPACE_ROOT="$2"; shift 2 ;;
    --report-dir)
      REPORT_DIR="$2"; shift 2 ;;
    *)
      echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

cd "$WORKSPACE_ROOT"
export WAVER_ALLOW_HARDWARE="${WAVER_ALLOW_HARDWARE:-0}"
export WAVER_NO_HARDWARE="${WAVER_NO_HARDWARE:-1}"
export WAVER_BLOCK_SERIAL="${WAVER_BLOCK_SERIAL:-1}"
export WAVER_DISABLE_SOUND_OUTPUT="${WAVER_DISABLE_SOUND_OUTPUT:-1}"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

bash scripts/waver_hardware_guard.sh --check-env

mkdir -p "${REPORT_DIR:-reports/quality_gate/manual_launch_dryrun}"
OUT_DIR="${REPORT_DIR:-reports/quality_gate/manual_launch_dryrun}"

python3 - "$OUT_DIR" <<'PY'
from __future__ import annotations

import importlib.util
import json
import pathlib
import sys
import traceback

report_dir = pathlib.Path(sys.argv[1])
report_dir.mkdir(parents=True, exist_ok=True)

launch_roots = [
    pathlib.Path("src/waver_patrol/launch"),
    pathlib.Path("src/ugv_main/ugv_gazebo/launch"),
    pathlib.Path("src/ugv_main/ugv_tools/launch"),
]
results = []
failures = 0
for root in launch_roots:
    if not root.exists():
        continue
    for path in sorted(root.glob("**/*.launch.py")):
        if any(part in {"build", "install", "log", "__pycache__"} for part in path.parts):
            continue
        item = {"path": str(path), "result": "PASS", "error": ""}
        spec = importlib.util.spec_from_file_location(path.stem.replace(".", "_"), path)
        module = importlib.util.module_from_spec(spec)
        try:
            assert spec.loader is not None
            spec.loader.exec_module(module)
            if not hasattr(module, "generate_launch_description"):
                item["result"] = "WARN"
                item["error"] = "generate_launch_description missing"
        except Exception:
            failures += 1
            item["result"] = "FAIL"
            item["error"] = traceback.format_exc()
        results.append(item)
        print(f"LAUNCH_DRYRUN_{item['result']} {path}")

summary = {
    "result": "PASS" if failures == 0 else "FAIL",
    "failures": failures,
    "checked": len(results),
    "results": results,
}
(report_dir / "launch_dryrun_report.json").write_text(json.dumps(summary, indent=2) + "\n")
(report_dir / "launch_dryrun_report.txt").write_text(
    "\n".join(
        [f"WAVER_LAUNCH_DRYRUN={summary['result']} checked={summary['checked']} failures={summary['failures']}"]
        + [f"- {r['result']} {r['path']}" for r in results]
    )
    + "\n"
)
raise SystemExit(0 if failures == 0 else 1)
PY
