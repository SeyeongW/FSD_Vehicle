import json
import subprocess
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]


def test_command_chain_dry_run_requires_collision_monitor_for_production(tmp_path):
    output = tmp_path / "cmd_chain.json"
    result = subprocess.run(
        [
            "python3",
            "scripts/waver_command_chain_check.py",
            "--dry-run",
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
    assert report["collision_monitor_required"] is True
    assert "/waver/cmd_vel_safety" in report["expected_chain"]


def test_production_profile_collision_monitor_policy():
    data = yaml.safe_load((ROOT / "config/real_profiles/bird_patrol_production.yaml").read_text())
    assert data["navigation"]["enable_collision_monitor"] is True
    assert data["navigation"]["collision_monitor_required_for_autonomous"] is True
    launch = (ROOT / "src/waver_patrol/launch/waver_nav2_radar_bird_mission.launch.py").read_text()
    assert "nav2_collision_monitor" in launch
    assert "collision_cmd_vel_in_topic" in launch
    assert "collision_cmd_vel_out_topic" in launch
