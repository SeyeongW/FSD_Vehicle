from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_release_hygiene_excludes_stale_runtime_reports():
    source_archive = (ROOT / "scripts/make_source_archive.py").read_text()
    checker = (ROOT / "scripts/check_bird_mission_field_release.py").read_text()
    for token in (
        "reports/livox_mid360/",
        "reports/camera/",
        "reports/bird_detector/",
        "reports/command_chain/latest",
        "reports/field_bridge_regression/latest",
        "reports/release_self_test/latest_",
    ):
        assert token in source_archive or token in checker
    assert "reports/README.md" in (ROOT / "scripts/make_bird_mission_field_release.py").read_text()
