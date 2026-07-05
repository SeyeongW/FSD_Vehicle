import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_fusion_live_fails_when_calibration_report_is_not_ready(tmp_path):
    livox = tmp_path / "livox.json"
    camera = tmp_path / "camera.json"
    detector = tmp_path / "detector.json"
    calib = tmp_path / "calib.json"
    out = tmp_path / "fusion.json"
    livox.write_text(json.dumps({"status": "LIDAR_READY", "pointcloud_rate_hz": 10, "point_count_mean": 2000}))
    camera.write_text(json.dumps({"status": "CAMERA_READY"}))
    detector.write_text(json.dumps({"status": "DETECTOR_READY", "class_map_ok": True, "max_latency_ms": 30}))
    calib.write_text(json.dumps({"status": "CALIBRATION_FAIL", "calibrated": False}))
    proc = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts/waver_bird_mission_readiness_check.py"),
            "--mode",
            "fusion-live",
            "--profile",
            str(ROOT / "config/real_profiles/bird_patrol_production.yaml"),
            "--strict",
            "--use-probe-reports",
            "--livox-report",
            str(livox),
            "--camera-report",
            str(camera),
            "--detector-report",
            str(detector),
            "--calibration-report",
            str(calib),
            "--output",
            str(out),
        ],
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode != 0
    assert json.loads(out.read_text())["status"] == "FAIL"
