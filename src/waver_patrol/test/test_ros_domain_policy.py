from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]


def test_quality_gate_and_network_config_default_to_ros_domain_zero():
    quality = (ROOT / "scripts/waver_quality_gate.sh").read_text()
    network = yaml.safe_load((ROOT / "config/network/waver_ros_domain.yaml").read_text())
    assert 'ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"' in quality
    assert network["default_ros_domain_id"] == 0
    assert network["field_ros_domain_id"] == 0
