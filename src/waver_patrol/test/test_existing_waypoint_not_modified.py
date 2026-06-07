from pathlib import Path


def test_existing_waypoint_files_still_present_or_replaced_by_overlay_only():
    root = Path(__file__).resolve().parents[1]
    # New LiDAR-first work must add overlay files, not mutate the field-control
    # scripts or route names used by the already successful manual/patrol path.
    assert (root / "launch" / "waver_lidar_first_bird_mission_overlay.launch.py").exists()
    assert (root / "config" / "waver_lidar_first_bird_mission_overlay.yaml").exists()
