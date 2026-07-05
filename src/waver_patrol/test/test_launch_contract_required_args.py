import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_launch_contract_checker_passes_static_contract(tmp_path):
    output = tmp_path / "launch_contract.json"
    result = subprocess.run(
        [
            "python3",
            "scripts/waver_launch_contract_check.py",
            "--profile",
            "config/real_profiles/bird_patrol_production.yaml",
            "--output",
            str(output),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(output.read_text())
    assert report["status"] == "PASS"
    assert not report["findings"]


def test_field_entrypoints_pass_bird_model_and_camera_extrinsic():
    start = (ROOT / "scripts/waver_bird_patrol_field_start.sh").read_text()
    backend = (ROOT / "scripts/waver_field_lidar_nav_backend_start.sh").read_text()
    assert "--bird-model" in start
    assert "--camera-extrinsic" in start
    assert "bird_model_path:=" in backend
    assert "camera_lidar_extrinsic:=" in backend
    assert "scan_topic:=" in backend
    assert "pointcloud_topic:=" in backend
