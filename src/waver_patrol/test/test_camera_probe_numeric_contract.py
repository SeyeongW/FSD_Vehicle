from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_camera_probe_reports_numeric_quality_fields():
    text = (ROOT / "scripts/waver_camera_probe.py").read_text()
    for token in (
        "image_rate_hz",
        "width",
        "height",
        "encoding",
        "image_frame_id",
        "camera_info_frame_id",
        "camera_info_has_k",
        "camera_info_has_d",
        "camera_info_has_p",
        "timestamp_monotonic",
        "frame_drop_count",
        "detected_topics",
    ):
        assert token in text
