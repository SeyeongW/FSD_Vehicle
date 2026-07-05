from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]


def test_hardware_acceptance_matrix_has_product_items_and_updater():
    matrix = yaml.safe_load((ROOT / "config/hardware_acceptance_matrix.yaml").read_text())
    names = {item["item"] for item in matrix["items"]}
    for required in (
        "livox_mid360_pointcloud",
        "camera_image",
        "camera_lidar_extrinsic",
        "base_driver_serial",
        "odom_feedback",
        "collision_monitor_active",
        "final_cmd_vel_single_publisher",
        "mission_mode_single_publisher",
        "bird_detector_model",
        "fusion_sync",
        "sound_backend",
        "emergency_stop",
        "blackbox",
        "battery_return",
        "departure_monitor",
    ):
        assert required in names
    readiness = (ROOT / "scripts/waver_bird_mission_readiness_check.py").read_text()
    updater = ROOT / "scripts/waver_hardware_acceptance_update.py"
    assert updater.exists()
    assert "--hardware-matrix" in readiness
    assert "hardware_matrix_required_items_pass" in readiness
