import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_source_mode_no_hardware_passes_and_writes_json(tmp_path):
    out = tmp_path / "readiness.json"
    proc = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts/waver_bird_mission_readiness_check.py"),
            "--mode",
            "source",
            "--profile",
            str(ROOT / "config/real_profiles/bird_patrol_production.yaml"),
            "--strict",
            "--no-hardware",
            "--output",
            str(out),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert "BIRD_MISSION_READINESS=PASS" in proc.stdout
    data = json.loads(out.read_text())
    assert data["mode"] == "source"
    assert data["level"] == "B0_SOURCE_AND_NAV_BASE"


def test_no_hardware_blocks_live_modes(tmp_path):
    out = tmp_path / "readiness.json"
    proc = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts/waver_bird_mission_readiness_check.py"),
            "--mode",
            "detector-live",
            "--profile",
            str(ROOT / "config/real_profiles/bird_patrol_production.yaml"),
            "--strict",
            "--no-hardware",
            "--output",
            str(out),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode != 0
    data = json.loads(out.read_text())
    assert data["blocked_capabilities"]
