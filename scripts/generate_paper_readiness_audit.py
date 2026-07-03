#!/usr/bin/env python3
from __future__ import annotations

import csv
import argparse
import json
import re
import subprocess
from datetime import datetime
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def cmd(args: list[str]) -> tuple[int, str]:
    try:
        p = subprocess.run(args, cwd=ROOT, text=True, capture_output=True, check=False)
        return p.returncode, (p.stdout + p.stderr).strip()
    except Exception as exc:
        return 99, str(exc)


def first_line(text: str) -> str:
    return text.splitlines()[0] if text.splitlines() else ""


def has_raw_data() -> bool:
    root = ROOT / "experiments_result"
    return root.exists() and any(root.rglob("experiment_summary.csv"))


def load_source_manifest() -> dict[str, object]:
    path = ROOT / "reports/source_manifest.json"
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def local_validation_summary() -> tuple[str, str]:
    path = ROOT / "reports" / "local_validation_summary.md"
    if path.exists():
        return "PASS_RECORDED", "`reports/local_validation_summary.md` records the latest static/package validation commands."
    return "NOT_RECORDED_BY_AUDIT", "Run compileall, shell syntax, unit tests, clone acceptance, archive dry-run, and contract check."


def repeated_claim_hits() -> list[str]:
    patterns = [
        r"\b10-run\b",
        r"\b10/10\b",
        r"\b10runs\b",
        r"precision mean:\s*1\.000",
        r"recall mean:\s*1\.000",
        r"\bn\s*=\s*5\b",
        r"\b5x\b",
        r"5회",
    ]
    targets = [ROOT / "README.md", ROOT / "docs", ROOT / "src/waver_patrol/docs"]
    hits: list[str] = []
    for target in targets:
        files = [target] if target.is_file() else list(target.rglob("*.md")) if target.exists() else []
        for path in files:
            text = path.read_text(errors="replace")
            for pattern in patterns:
                if re.search(pattern, text, flags=re.IGNORECASE):
                    hits.append(f"{path.relative_to(ROOT)}:{pattern}")
    return hits


def has_current_schema_smoke_pass() -> bool:
    for summary in sorted((ROOT / "reports").glob("gazebo_functional_validation/*/summary.csv")):
        try:
            with summary.open(newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                if {"raw_status", "final_status", "limitation", "command_rc", "evidence_path", "key_output"} <= set(reader.fieldnames or []):
                    rows = list(reader)
                    if rows and all(row.get("final_status") == "PASS" for row in rows):
                        return True
        except Exception:
            continue
    return False


def write_manifest(path: Path) -> None:
    fields = [
        "artifact_path",
        "artifact_type",
        "scenario",
        "trial_id",
        "evidence_level",
        "sim_or_real",
        "fake_detector_used",
        "fake_sound_used",
        "serial_enabled",
        "ground_truth_source",
        "status",
        "status_source",
        "notes",
    ]
    rows: list[dict[str, str]] = []
    exp = ROOT / "experiments_result"
    if not exp.exists():
        rows.append(
            {
                "artifact_path": "experiments_result/",
                "artifact_type": "raw_experiment_dir",
                "scenario": "N/A",
                "trial_id": "N/A",
                "evidence_level": "NONE",
                "sim_or_real": "N/A",
                "fake_detector_used": "UNKNOWN",
                "fake_sound_used": "UNKNOWN",
                "serial_enabled": "UNKNOWN",
                "ground_truth_source": "N/A",
                "status": "MISSING_RAW_EXPERIMENT_DATA",
                "status_source": "filesystem",
                "notes": "No raw experiment package is present.",
            }
        )
    else:
        for csv_path in sorted(exp.rglob("experiment_summary.csv")):
            rows.append(
                {
                    "artifact_path": str(csv_path.relative_to(ROOT)),
                    "artifact_type": "experiment_summary_csv",
                    "scenario": "UNKNOWN",
                    "trial_id": "UNKNOWN",
                    "evidence_level": "L2_GAZEBO_OR_UNKNOWN",
                    "sim_or_real": "UNKNOWN",
                    "fake_detector_used": "UNKNOWN",
                    "fake_sound_used": "UNKNOWN",
                    "serial_enabled": "UNKNOWN",
                    "ground_truth_source": "UNKNOWN",
                    "status": "PRESENT_UNAUDITED",
                    "status_source": "filesystem",
                    "notes": "Run prepare_paper_results.py for table-level audit.",
                }
            )
    for report in sorted((ROOT / "reports").glob("**/summary.csv")):
        rows.append(
            {
                "artifact_path": str(report.relative_to(ROOT)),
                "artifact_type": "validation_summary_csv",
                "scenario": "N/A",
                "trial_id": "N/A",
                "evidence_level": "L0_L2_DEPENDS_ON_REPORT",
                "sim_or_real": "sim_or_static",
                "fake_detector_used": "UNKNOWN",
                "fake_sound_used": "UNKNOWN",
                "serial_enabled": "false_or_unknown",
                "ground_truth_source": "N/A",
                "status": "LEGACY_OR_UNAUDITED_NOT_FOR_CLAIMS",
                "status_source": "reports",
                "notes": "Do not treat SKIP_WITH_REASON as PASS.",
            }
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate Waver paper/source readiness audit.")
    parser.add_argument(
        "--paper-mode",
        choices=("source_only", "simulation_smoke", "experimental_evidence"),
        default="source_only",
        help="Target paper readiness mode. Repeated-trial performance claims are not inferred.",
    )
    args = parser.parse_args()

    source_manifest = load_source_manifest()
    has_git = (ROOT / ".git").exists()
    branch_rc, branch = cmd(["git", "branch", "--show-current"]) if has_git else (1, "")
    hash_rc, commit = cmd(["git", "rev-parse", "--short", "HEAD"]) if has_git else (1, "")
    sub_rc, sub = cmd(["git", "submodule", "status", "--recursive"]) if has_git else (1, "")
    status_rc, status = cmd(["git", "status", "--short"]) if has_git else (1, "")
    if not branch:
        branch = str(source_manifest.get("git_branch_or_unknown", "UNKNOWN"))
    if not commit:
        commit = str(source_manifest.get("git_commit_or_unknown", "UNKNOWN"))
    raw_present = has_raw_data()
    claim_hits = repeated_claim_hits()
    code_status, code_notes = local_validation_summary()
    dirty_lines = [line for line in status.splitlines() if line]
    dirty_count = sum(1 for line in dirty_lines if not line.startswith("??"))
    untracked_count = sum(1 for line in dirty_lines if line.startswith("??"))

    blockers: list[str] = []
    limitations: list[str] = []
    if branch != "jo":
        blockers.append("branch is not jo")
    if code_status != "PASS_RECORDED":
        blockers.append("local static/package validation summary is missing")
    if claim_hits:
        blockers.append("repeated-trial or perfect-performance claim text remains in paper-facing docs")
    if "src/livox_ros_driver2" in status or any(line.startswith(" m ") or line.startswith("+") or line.startswith("-") for line in sub.splitlines()):
        limitations.append("livox_ros_driver2 submodule has local/dirty changes; patch policy B is documented")
    if not raw_present:
        limitations.append("raw experiment data absent; experimental performance claims are unavailable")
    if dirty_lines:
        limitations.append("working tree has dirty/untracked files; HEAD commit alone does not reproduce this artifact")
    if not source_manifest:
        limitations.append("source_manifest.json has not yet been generated by make_source_archive.py")

    smoke_pass = has_current_schema_smoke_pass()
    if args.paper_mode == "simulation_smoke" and not smoke_pass:
        blockers.append("no current-schema Gazebo single-run smoke summary with final_status=PASS")
    if args.paper_mode == "experimental_evidence" and not raw_present:
        blockers.append("experimental evidence mode requested but raw experiment data is absent")

    if blockers:
        final_status = "NOT_PAPER_READY"
    elif args.paper_mode == "experimental_evidence":
        final_status = "PAPER_READY_WITH_EXPERIMENTAL_EVIDENCE"
    elif args.paper_mode == "simulation_smoke":
        final_status = "PAPER_READY_SIMULATION_SMOKE"
    else:
        final_status = "PAPER_READY_SOURCE_ONLY"

    write_manifest(ROOT / "reports" / "paper_evidence_manifest.csv")
    text = [
        "# Paper Readiness Audit",
        "",
        f"Final judgement: **{final_status}**",
        "",
        f"- Generated at: {datetime.now().isoformat(timespec='seconds')}",
        f"- Repository root: `{ROOT}`",
        f"- Paper mode: `{args.paper_mode}`",
        f"- Branch: `{branch if branch_rc == 0 else 'UNKNOWN'}`",
        f"- Commit: `{commit if hash_rc == 0 else 'UNKNOWN'}`",
        f"- Dirty file count: `{dirty_count}`",
        f"- Untracked file count: `{untracked_count}`",
        f"- Raw experiment data present: `{raw_present}`",
        f"- Source manifest present: `{bool(source_manifest)}`",
        "",
        "## Blocking Reasons",
        "",
        *([f"- {reason}" for reason in blockers] or ["- none for the selected source-only policy"]),
        "",
        "## Limitations / Non-Blockers",
        "",
        *([f"- {reason}" for reason in limitations] or ["- none recorded"]),
        "",
        "## Evidence Summary",
        "",
        "| Item | Status | Notes |",
        "|---|---|---|",
        f"| Code validation | {code_status} | {code_notes} |",
        f"| Gazebo validation | {'PASS_RECORDED' if smoke_pass else 'NOT_PROVEN_BY_THIS_AUDIT'} | Current-schema single-run smoke PASS is required for simulation-smoke claims. |",
        f"| Real wheel-on readiness | NOT_PROVEN | No automatic motor/serial/wheel-on execution is performed by this audit. |",
        f"| Paper raw data | {'PRESENT_UNAUDITED' if raw_present else 'ABSENT_PERFORMANCE_CLAIM_UNAVAILABLE'} | Raw data is required only for experimental performance claims. |",
        f"| Livox/MID-360 sim plugin | LIMITED | Dirty submodule patch captured at `patches/livox_ros_driver2_humble_mid360.patch`. |",
        f"| Repeated performance claims | {'BLOCKED' if claim_hits else 'NONE_FOUND'} | This package does not claim repeated-count or perfect-metric performance. |",
        "",
        "## Source Manifest",
        "",
        f"- Manifest path: `reports/source_manifest.json`",
        f"- SHA256 manifest path: `reports/source_sha256_manifest.csv`",
        f"- Manifest artifact type: `{source_manifest.get('artifact_type', 'UNKNOWN') if source_manifest else 'UNKNOWN'}`",
        f"- Manifest generated at: `{source_manifest.get('generated_at', 'UNKNOWN') if source_manifest else 'UNKNOWN'}`",
        f"- Manifest git branch: `{source_manifest.get('git_branch_or_unknown', 'UNKNOWN') if source_manifest else 'UNKNOWN'}`",
        f"- Manifest git commit: `{source_manifest.get('git_commit_or_unknown', 'UNKNOWN') if source_manifest else 'UNKNOWN'}`",
        f"- Manifest included file count: `{source_manifest.get('included_file_count', 'UNKNOWN') if source_manifest else 'UNKNOWN'}`",
        "",
        "HEAD commit alone does not reproduce this artifact when the working tree is dirty or has untracked files; use the source manifest and sha256 manifest for artifact-level reproducibility.",
        "",
        "## Repeated-Claim Scan",
        "",
        *([f"- {hit}" for hit in claim_hits] or ["- none found in paper-facing markdown docs"]),
        "",
        "## Submodule Status",
        "",
        "```text",
        sub if sub_rc == 0 and sub else "No submodule status available.",
        "```",
        "",
        "## Claim Boundary",
        "",
        "- Allowed: static/unit contract checks, Gazebo smoke or simulation claims only when logs are PASS.",
        "- Forbidden: real wheel-on safety, real bird detection accuracy, real deterrence effect, or Livox simulation success when plugin logs show failure.",
        "",
        "See `reports/paper_evidence_manifest.csv` for artifact-level evidence rows.",
    ]
    out = ROOT / "reports" / "paper_readiness_audit.md"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(text) + "\n", encoding="utf-8")
    print(f"PAPER_READINESS_AUDIT={out}")
    print(f"PAPER_EVIDENCE_MANIFEST={ROOT / 'reports' / 'paper_evidence_manifest.csv'}")
    print(f"FINAL_JUDGEMENT={final_status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
