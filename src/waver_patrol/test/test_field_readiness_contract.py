import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_field_readiness_script_supports_no_hardware_l1(tmp_path):
    output = tmp_path / "l1_readiness.json"
    result = subprocess.run(
        [
            "python3",
            str(ROOT / "scripts/waver_field_readiness_check.py"),
            "--level",
            "L1",
            "--strict",
            "--no-hardware",
            "--output",
            str(output),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "FIELD_READINESS=PASS" in result.stdout
    report = json.loads(output.read_text())
    assert report["level"] == "L1"
    assert report["status"] == "PASS"
    assert any(row["name"] == "dry_run_no_hardware_guard" for row in report["required_checks"])


def test_field_backend_does_not_print_unconditional_ready():
    script = text("scripts/waver_field_lidar_nav_backend_start.sh")
    assert "waver_field_readiness_check.py" in script
    assert "LIDAR_NAV_BACKEND_READY=PASS|PASS_LIMITED|FAIL" in script
    assert "LIDAR_NAV_BACKEND_READY=YES" not in script
    assert "FIELD_READINESS_STRICT" in script


def test_readiness_levels_are_fail_closed():
    doc = text("docs/hardware_readiness_levels.md")
    checker = text("scripts/waver_field_readiness_check.py")
    for level in ("L0", "L1", "L2", "L3", "L4", "L5"):
        assert level in doc
    assert "--no-hardware can only PASS L0/L1 checks" in checker
    assert "WAVER_ACK_SOUND_HARDWARE" in checker
    assert "WAVER_ACK_LOCAL_SOUND_LAW" in checker
