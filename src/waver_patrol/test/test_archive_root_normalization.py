from pathlib import Path

import importlib.util


ROOT = Path(__file__).resolve().parents[3]


def load_checker():
    path = ROOT / "scripts/check_bird_mission_field_release.py"
    spec = importlib.util.spec_from_file_location("check_bird_mission_field_release", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_release_archive_root_normalizes_ros2_ws5_fsd_vehicle_prefix():
    checker = load_checker()
    raw = [
        "ros2_ws5/FSD_Vehicle/README_BIRD_PATROL_FIELD.md",
        "ros2_ws5/FSD_Vehicle/src/waver_patrol/package.xml",
        "ros2_ws5/FSD_Vehicle/scripts/waver_bird_patrol_field_start.sh",
    ]
    names = checker.normalize_archive_names(raw)
    assert "README_BIRD_PATROL_FIELD.md" in names
    assert "src/waver_patrol/package.xml" in names
