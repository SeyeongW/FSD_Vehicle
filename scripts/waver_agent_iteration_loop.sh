#!/usr/bin/env bash
set -euo pipefail

MAX_ITERATIONS=3
TIME_BUDGET_HOURS=2
NO_HARDWARE=false
STOP_ON_CRITICAL=true
REPORT_ROOT="reports/agent_iterations"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --max-iterations)
      MAX_ITERATIONS="$2"; shift 2 ;;
    --time-budget-hours)
      TIME_BUDGET_HOURS="$2"; shift 2 ;;
    --no-hardware)
      NO_HARDWARE=true; shift ;;
    --stop-on-critical-safety-regression)
      STOP_ON_CRITICAL=true; shift ;;
    --report-root)
      REPORT_ROOT="$2"; shift 2 ;;
    *)
      echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [ "$NO_HARDWARE" != "true" ]; then
  echo "ERROR: iteration loop requires --no-hardware." >&2
  exit 2
fi

START_EPOCH="$(date +%s)"
BUDGET_SEC="$(python3 - <<PY
print(int(float("$TIME_BUDGET_HOURS") * 3600))
PY
)"
mkdir -p "$REPORT_ROOT"
PREV_SCORE=""

for i in $(seq 1 "$MAX_ITERATIONS"); do
  now="$(date +%s)"
  if [ $((now - START_EPOCH)) -ge "$BUDGET_SEC" ]; then
    echo "TIME_BUDGET_EXHAUSTED before iteration=$i"
    break
  fi
  stamp="$(date +%Y%m%d_%H%M%S)"
  iter_dir="$REPORT_ROOT/${stamp}_iter_${i}"
  mkdir -p "$iter_dir"
  echo "ITERATION=$i REPORT_DIR=$iter_dir"
  git status --short > "$iter_dir/git_status_before.txt" 2>&1 || true
  set +e
  bash scripts/waver_quality_gate.sh --no-hardware --report-dir "$iter_dir/quality_gate"
  gate_status=$?
  set -e
  git status --short > "$iter_dir/git_status_after.txt" 2>&1 || true
  python3 - "$iter_dir" "$gate_status" "$PREV_SCORE" <<'PY'
import json
import pathlib
import sys

iter_dir = pathlib.Path(sys.argv[1])
gate_status = int(sys.argv[2])
prev_score_raw = sys.argv[3]
summary_path = iter_dir / "quality_gate" / "quality_gate_summary.json"
contract_path = iter_dir / "quality_gate" / "contract_report.json"
summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
contract = json.loads(contract_path.read_text()) if contract_path.exists() else {}
score = int(summary.get("score", 999999))
prev = int(prev_score_raw) if prev_score_raw else None
critical = int(contract.get("counts", {}).get("CRITICAL", 0)) if contract else 0
high = int(contract.get("counts", {}).get("HIGH", 0)) if contract else 0
next_issues = contract.get("next_recommended_fixes", []) if contract else []
lines = [
    f"iteration_status={'PASS' if gate_status == 0 else 'FAIL'}",
    f"score={score}",
    f"previous_score={prev if prev is not None else 'none'}",
    f"score_delta={(score - prev) if prev is not None else 'n/a'}",
    f"critical_issues={critical}",
    f"high_issues={high}",
    "",
    "Next recommended fixes:",
]
for item in next_issues[:10]:
    lines.append(f"- {item}")
if not next_issues:
    lines.append("- No CRITICAL/HIGH contract issues reported.")
(iter_dir / "iteration_summary.md").write_text("\n".join(lines) + "\n")
(iter_dir / "next_issue.md").write_text("\n".join(f"- {x}" for x in next_issues[:10]) + "\n")
print(f"ITERATION_SCORE={score}")
print(f"ITERATION_CRITICAL={critical}")
print(f"ITERATION_HIGH={high}")
PY
  current_score="$(python3 -c 'import json,sys,pathlib; p=pathlib.Path(sys.argv[1]); print(json.loads(p.read_text()).get("score", 999999) if p.exists() else 999999)' "$iter_dir/quality_gate/quality_gate_summary.json")"
  PREV_SCORE="$current_score"
  if [ "$STOP_ON_CRITICAL" = "true" ] && [ -f "$iter_dir/quality_gate/contract_report.json" ]; then
    critical="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("counts",{}).get("CRITICAL",0))' "$iter_dir/quality_gate/contract_report.json")"
    if [ "$critical" -gt 0 ]; then
      echo "STOP_ON_CRITICAL critical=$critical"
      exit 1
    fi
  fi
done

