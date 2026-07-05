from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_calibration_checker_supports_live_tf_topic_contract():
    text = (ROOT / "scripts/waver_camera_lidar_calibration_check.py").read_text()
    for token in (
        "--extrinsic",
        "--camera-info-topic",
        "--pointcloud-topic",
        "--require-live-tf",
        "--require-live-topics",
        "camera_info_frame_id",
        "pointcloud_frame_id",
        "tf_base_to_lidar_ready",
        "tf_base_to_camera_ready",
        "tf_camera_to_lidar_ready",
    ):
        assert token in text
