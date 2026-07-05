import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_worktree_inventory_script_classifies_generated_and_field_files(tmp_path):
    output = tmp_path / "inventory.md"
    result = subprocess.run(
        ["python3", "scripts/waver_worktree_inventory.py", "--output", str(output)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    report = output.read_text()
    assert "build/`, `install/`, and `log/` are generated artifacts" in report
    assert "FIELD_BRIDGE_CRITICAL" in report
    assert "FIELD_RELEASE_REQUIRED" in report
    assert "VENDOR_OR_SUBMODULE" in report or "src/livox_ros_driver2" not in report


def test_worktree_inventory_knows_required_bird_production_files():
    script = (ROOT / "scripts/waver_worktree_inventory.py").read_text()
    for token in [
        "README_BIRD_PATROL_FIELD.md",
        "config/real_profiles/bird_patrol_production.yaml",
        "scripts/waver_bird_patrol_field_start.sh",
        "src/waver_patrol/launch/bird_patrol_production.launch.py",
    ]:
        assert token in script
