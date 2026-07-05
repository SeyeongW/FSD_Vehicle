# UI SLAM + Bird Detection Smoke Reports

This directory stores reports from:

```bash
bash scripts/run_ui_slam_bird_detection_gazebo_smoke.sh
```

The smoke test is simulation-only. It verifies that the remote panel can keep
SLAM mapping visible while bird/perception status topics are also live.

It must not be used as real hardware evidence.

Expected latest report:

```text
reports/ui_slam_bird_detection/latest.json
```

Required pass fields:

- `current_map_source_slam_live`
- `map_publisher_count == 1`
- `cmd_vel_publisher_count == 1`
- `mode_publisher_count == 1`
- `bird_topics_visible`
- `bird_topics_fresh`
- `bird_detector_state_fresh`
- `bird_fusion_state_fresh`
- `mapping_path_visible`
- `no_patrol_conflict`
- `no_target_approach_without_arm`
- `no_sound_without_arm`
- `map_quality_pass`

This test is separate from real field readiness. Real readiness still requires
Jetson, Livox, camera, calibration, base feedback, E-stop, and safety evidence.
