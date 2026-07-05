import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_cleanup_policy_document_preserves_field_bridge_scripts():
    policy = (ROOT / "docs/repository_cleanup_policy.md").read_text(encoding="utf-8")
    for rel in [
        "scripts/waver_field_docker_backend_start.sh",
        "scripts/waver_field_local_ui_start.sh",
        "scripts/waver_field_lidar_nav_backend_start.sh",
        "scripts/waver_start_field_backend.sh",
    ]:
        assert rel in policy
    assert "must never be automatic cleanup candidates" in policy
    assert "SOURCE_REQUIRED" in policy
    assert "LOCAL_RUNTIME_REQUIRED" in policy
    assert "VENDOR_OR_SUBMODULE" in policy


def test_cleanup_plan_script_dry_run_generates_report(tmp_path):
    output = tmp_path / "cleanup_plan.md"
    result = subprocess.run(
        ["python3", "scripts/waver_conservative_cleanup_plan.py", "--output", str(output)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    report = output.read_text(encoding="utf-8")
    assert "Waver Conservative Cleanup Plan" in report
    assert "LOCAL_RUNTIME_REQUIRED" in report
    assert "GENERATED_ARTIFACT" in report
    assert "scripts/waver_field_local_ui_start.sh" in report


def test_cleanup_plan_apply_is_restricted_to_generated_artifacts():
    script = (ROOT / "scripts/waver_conservative_cleanup_plan.py").read_text(encoding="utf-8")
    assert "AUTO_APPLY_DIRS" in script
    assert '"build"' in script
    assert '"install"' in script
    assert '"log"' in script
    assert "FIELD_BRIDGE_SCRIPTS" in script
    assert "category != \"GENERATED_ARTIFACT\"" in script
    assert "SOURCE_REQUIRED" in script
