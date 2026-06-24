from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_hardware_free_docs_exist():
    for rel in [
        "AGENT_LOOP.md",
        "QUALITY_GATES.md",
        "REAL_VEHICLE_TEST_PLAN.md",
        "REAL_TEST_REPORT_TEMPLATE.md",
        "ROSBAG_REGRESSION_GUIDE.md",
        "reports/README.md",
    ]:
        assert (ROOT / rel).exists(), rel


def test_hardware_guard_blocks_obvious_real_hardware_tokens():
    text = (ROOT / "scripts/waver_hardware_guard.sh").read_text()
    assert "start_serial_bridge:=true" in text
    assert "enable_waver_base_driver:=true" in text
    assert "/dev/serial/by-id" in text
    assert "enable_sound_output:=true" in text
