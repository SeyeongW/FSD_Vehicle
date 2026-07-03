import yaml
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def text(path: str) -> str:
    return (ROOT / path).read_text(errors="replace")


def test_base_feedback_schema_has_required_fields():
    schema = yaml.safe_load((ROOT / "config/waver_base_feedback_schema.yaml").read_text())
    assert schema["packet_type"] == 1001
    assert schema["left_odom_field"] == "odl"
    assert schema["right_odom_field"] == "odr"
    assert schema["unit"] in {"m", "cm", "mm", "tick", "ticks"}
    assert isinstance(schema["scale_left"], (int, float))
    assert isinstance(schema["scale_right"], (int, float))
    assert isinstance(schema["invert_left"], bool)
    assert isinstance(schema["invert_right"], bool)


def test_base_driver_reports_schema_feedback_state():
    driver = text("src/waver_patrol/waver_patrol/bridges/waver_base_driver_node.py")
    for token in (
        "feedback_schema_path",
        "ODOM_FEEDBACK_OK",
        "ODOM_FEEDBACK_MISSING",
        "ODOM_FEEDBACK_STALE",
        "ODOM_FEEDBACK_SCHEMA_ERROR",
        "left_odom_field",
        "right_odom_field",
        "odom_feedback_status",
    ):
        assert token in driver
    assert "handle_odom_feedback" in driver
    assert "last_odom_feedback_time" in driver
