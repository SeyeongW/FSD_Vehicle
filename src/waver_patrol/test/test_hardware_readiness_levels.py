from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_hardware_readiness_level_table_names_required_levels():
    doc = (ROOT / "docs/hardware_readiness_levels.md").read_text(errors="replace")
    for level in (
        "L0_SOURCE_CHECK",
        "L1_ROS_GRAPH_DRY_RUN",
        "L2_SENSOR_ONLY_LIVE",
        "L3_WHEEL_OFF_DRIVER",
        "L4_WHEEL_ON_LOW_SPEED",
        "L5_AUTONOMOUS_PATROL",
    ):
        assert level in doc
    assert "serial disabled" in doc
    assert "human E-stop" in doc


def test_hardware_acceptance_matrix_is_machine_readable_for_l4():
    matrix = (ROOT / "docs/hardware_acceptance_matrix.md").read_text(errors="replace")
    assert "| item | required_for_level | evidence_command | evidence_file | status | notes |" in matrix
    assert "battery voltage scale verified" in matrix
    assert "map -> odom localization verified" in matrix
    assert "E-stop physically verified" in matrix
    assert "TODO" in matrix
