import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_field_bridge_regression_dry_run_passes(tmp_path):
    output = tmp_path / "bridge.json"
    result = subprocess.run(
        ["python3", "scripts/waver_field_bridge_regression_check.py", "--dry-run", "--output", str(output)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    report = json.loads(output.read_text())
    assert report["status"] == "PASS"
    assert "local PC -> SSH -> Jetson host -> docker exec" in report["field_bridge"]
    assert report["ros_domain_id"]
    assert "docker exec" in report["docker_exec_command"]
    assert report["product_launch_target"] == "waver_patrol bird_patrol_production.launch.py"
    assert "/waver/manual_cmd_vel" in report["expected_command_chain"]


def test_local_ui_bridge_does_not_publish_final_cmd_vel_directly():
    ui = (ROOT / "scripts/waver_field_local_ui_start.sh").read_text()
    assert "remote_bridge_enabled:=true" in ui
    assert "publish_direct_cmd_vel:=false" in ui
    assert "remote_bridge_ros_domain_id" in ui
