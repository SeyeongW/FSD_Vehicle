from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[3]
FUSION = ROOT / "src/waver_patrol/waver_patrol/perception/bird_3d_fusion_node.py"


def test_fusion_uses_sync_and_standard_invalid_reasons():
    text = FUSION.read_text()
    assert "ApproximateTimeSynchronizer" in text
    for token in (
        "FUSION_VALID",
        "FUSION_INVALID_NO_BIRD",
        "FUSION_INVALID_NO_EXTRINSIC",
        "FUSION_INVALID_CALIBRATION_NOT_VERIFIED",
        "FUSION_INVALID_SYNC_STALE",
        "FUSION_INVALID_INSUFFICIENT_POINTS",
        "FUSION_INVALID_SPREAD_TOO_LARGE",
        "FUSION_INVALID_HEIGHT",
        "FUSION_INVALID_DYNAMIC_ASSOCIATION",
        "FUSION_INVALID_TF_FAIL",
        "/waver/bird_fusion_sync_state",
    ):
        assert token in text


def test_default_camera_lidar_calibration_is_not_claimed_ready():
    data = yaml.safe_load((ROOT / "config/sensors/camera_lidar_extrinsic.yaml").read_text())
    assert data["calibrated"] is False
    assert data["camera_frame"]
    assert data["lidar_frame"]
