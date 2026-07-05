import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CHECKER = ROOT / "scripts/waver_bird_mission_readiness_check.py"


def test_sensor_live_with_missing_probe_reports_fails(tmp_path):
    out = tmp_path / "sensor.json"
    proc = subprocess.run(
        [
            "python3",
            str(CHECKER),
            "--mode",
            "sensor-live",
            "--profile",
            str(ROOT / "config/real_profiles/bird_patrol_production.yaml"),
            "--strict",
            "--use-probe-reports",
            "--livox-report",
            str(tmp_path / "missing_livox.json"),
            "--camera-report",
            str(tmp_path / "missing_camera.json"),
            "--output",
            str(out),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode != 0
    assert json.loads(out.read_text())["status"] == "FAIL"


def test_autonomous_collision_monitor_missing_fails(tmp_path):
    out = tmp_path / "auto.json"
    proc = subprocess.run(
        [
            "python3",
            str(CHECKER),
            "--mode",
            "autonomous-patrol",
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
    assert json.loads(out.read_text())["status"] == "FAIL"
