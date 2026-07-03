from __future__ import annotations

import csv
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "src/waver_patrol/scripts/prepare_paper_results.py"


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def test_prepare_paper_results_accepts_single_fixture_without_n5_gate(tmp_path: Path) -> None:
    input_dir = tmp_path / "experiments_result"
    output = tmp_path / "paper_ready"
    _write_csv(
        input_dir / "run_H1" / "experiment_summary.csv",
        [
            {
                "trial_id": "1",
                "scenario": "H1_elevated_dynamic",
                "expected_elevated_dynamic_target_valid": "true",
                "elevated_dynamic_target_valid": "true",
                "target_goal_success": "true",
                "safety_gate_pass": "true",
                "overall_success": "true",
                "fake_detector_used": "true",
                "fake_sound_used": "true",
                "sim_or_real": "sim",
            }
        ],
    )
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input-dir",
            str(input_dir),
            "--output-root",
            str(output),
            "--paper-strict",
            "--no-latest",
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    metrics = next(output.glob("*/tables/paper_metrics.csv"))
    assert metrics.exists()
    assert "overall_success_rate" in metrics.read_text(encoding="utf-8")


def test_prepare_paper_results_fails_when_raw_data_missing(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            "--input-dir",
            str(tmp_path / "missing_experiments_result"),
            "--output-root",
            str(tmp_path / "paper_ready"),
            "--paper-strict",
            "--no-latest",
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode != 0
    assert "MISSING_RAW_EXPERIMENT_DATA" in result.stdout
