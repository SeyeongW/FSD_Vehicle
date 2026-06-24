#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
BAG_PATH=""
OPTIONAL=false
REPORT_DIR="${REPORT_DIR:-}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --root)
      WORKSPACE_ROOT="$2"; shift 2 ;;
    --bag)
      BAG_PATH="$2"; shift 2 ;;
    --optional)
      OPTIONAL=true; shift ;;
    --report-dir)
      REPORT_DIR="$2"; shift 2 ;;
    --replay)
      echo "ERROR: this checker does not replay by default. Use offline test nodes for replay regression." >&2
      exit 3 ;;
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

OUT_DIR="${REPORT_DIR:-reports/quality_gate/manual_rosbag_replay}"
mkdir -p "$OUT_DIR"

if [ -z "$BAG_PATH" ]; then
  if [ "$OPTIONAL" = "true" ]; then
    {
      echo "WAVER_ROSBAG_REPLAY_CHECK=SKIP"
      echo "reason=no bag path supplied; optional mode"
    } | tee "$OUT_DIR/rosbag_replay_check.txt"
    printf '{"result":"SKIP","reason":"no bag path supplied; optional mode"}\n' > "$OUT_DIR/rosbag_replay_check.json"
    exit 0
  fi
  echo "ERROR: --bag is required unless --optional is used." >&2
  exit 2
fi

if [ ! -e "$BAG_PATH" ]; then
  echo "ERROR: bag path does not exist: $BAG_PATH" >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 command not found." >&2
  exit 2
fi

set +e
ros2 bag info "$BAG_PATH" > "$OUT_DIR/rosbag_info.txt" 2>&1
STATUS=$?
set -e
cat "$OUT_DIR/rosbag_info.txt"

python3 - "$OUT_DIR" "$STATUS" <<'PY'
import json
import pathlib
import sys

out = pathlib.Path(sys.argv[1])
status = int(sys.argv[2])
text = (out / "rosbag_info.txt").read_text(errors="replace") if (out / "rosbag_info.txt").exists() else ""
required = ["/cmd_vel", "/waver/safety_state", "/waver/mode"]
missing = [topic for topic in required if topic not in text]
result = "PASS" if status == 0 and not missing else "FAIL"
summary = {
    "result": result,
    "ros2_bag_info_status": status,
    "required_topics_missing": missing,
}
(out / "rosbag_replay_check.json").write_text(json.dumps(summary, indent=2) + "\n")
(out / "rosbag_replay_check.txt").write_text(
    f"WAVER_ROSBAG_REPLAY_CHECK={result}\nmissing={missing}\n"
)
print(f"WAVER_ROSBAG_REPLAY_CHECK={result}")
raise SystemExit(0 if result == "PASS" else 1)
PY
