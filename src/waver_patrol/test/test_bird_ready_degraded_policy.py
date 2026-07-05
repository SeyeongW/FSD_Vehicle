import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CHECKER = ROOT / "scripts/waver_bird_mission_readiness_check.py"


def run_checker(tmp_path, mode: str, *extra: str):
    out = tmp_path / f"{mode}.json"
    proc = subprocess.run(
        [
            "python3",
            str(CHECKER),
            "--mode",
            mode,
            "--profile",
            str(ROOT / "config/real_profiles/bird_patrol_production.yaml"),
            "--strict",
            "--no-hardware",
            "--output",
            str(out),
            *extra,
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    return proc, json.loads(out.read_text())


def test_detector_live_no_hardware_fails_not_degraded(tmp_path):
    proc, data = run_checker(tmp_path, "detector-live")
    assert proc.returncode != 0
    assert data["status"] == "FAIL"


def test_autonomous_patrol_never_reports_pass_degraded(tmp_path):
    proc, data = run_checker(tmp_path, "autonomous-patrol")
    assert proc.returncode != 0
    assert data["status"] != "PASS_DEGRADED"


def test_audit_does_not_promote_degraded_to_autonomous(tmp_path):
    report_dir = ROOT / "reports/bird_mission_readiness"
    report_dir.mkdir(parents=True, exist_ok=True)
    probe = report_dir / "0000000000_autonomous-patrol_test.json"
    probe.write_text(json.dumps({"mode": "autonomous-patrol", "status": "PASS_DEGRADED"}) + "\n")
    try:
        proc = subprocess.run(
            ["python3", str(ROOT / "scripts/generate_bird_mission_readiness_audit.py")],
            text=True,
            capture_output=True,
            check=False,
        )
        assert proc.returncode == 0
        audit = (ROOT / "reports/bird_mission_readiness_audit.md").read_text()
        assert not audit.startswith("BIRD_PATROL_AUTONOMOUS_READY")
    finally:
        probe.unlink(missing_ok=True)
