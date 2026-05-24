# experiments_result Schema

All pre-real validation outputs default to:

```text
~/ros2_ws/experiments_result/
```

This directory is ignored by Git because it can contain rosbag databases, plots,
screenshots, and repeated trial CSV files.  Selected summary tables can be copied
into documentation later, but raw bags and large trial folders must stay local.

## Main Layout

```text
experiments_result/
  run_logs/
  pre_real_height_H1_<timestamp>/
    experiment_summary.csv
    results/
      pre_real_success_rate.csv
      pre_real_validation_report.md
      target_height_plot.png
      mission_success_plot.png
  pre_real_height_H2_<timestamp>/
  pre_real_height_H3_<timestamp>/
  ui_validation_<timestamp>/
    csv/
      ui_visualization_check.csv
    reports/
      ui_visualization_check.md
    experiment_summary.csv
  results/
    gazebo_trial_summary.csv
    gazebo_success_rate.csv
    target_height_plot.png
    mission_success_plot.png
    pre_real_safety_gate_result.png
    final_pass_fail_report.md
  paper_ready/
    latest -> paper_<timestamp>/
    paper_<timestamp>/
      tables/
        height_target_trials_selected.csv
        height_target_trials_all.csv
        ui_visualization_selected.csv
        ui_visualization_all.csv
        paper_metrics.csv
      figures/
        paper_target_height_by_trial.png
        paper_target_validity_by_trial.png
        paper_ui_success_by_trial.png
      reports/
        paper_results_summary.md
      raw_selected/
        *_experiment_summary.csv
      source_manifest.csv
```

`results/` is a convenience folder for recent merged analysis.  For thesis or
paper writing, use `paper_ready/latest/` because it separates final selected
tables, figures, and report from raw repeated trial folders.

## Required Height Columns

`experiment_summary.csv` and merged `gazebo_trial_summary.csv` include:

- `target_min_height_m`: default 3.0 m.
- `target_object_height_m`: object z in the height reference frame after ground offset.
- `z_valid`: false for 2D-only detections.
- `height_filter_pass`: true only when object height is within the target band.
- `dynamic_filter_pass`: true only when map/odom compensated motion or velocity is above threshold.
- `elevated_dynamic_target_valid`: final target gate used by mission logic.
- `classification`: `elevated_dynamic_object`, `unknown_or_static`, `low_altitude_object`, or related rejection reason.
- `compensated_motion_m`: motion measured in map/odom frame, not raw LiDAR frame.

## UI Validation Columns

`ui_visualization_check.csv` includes:

- `map_received`
- `map_mode`
- `slam_live`
- `map_fixed`
- `robot_pose_visible`
- `robot_yaw_visible`
- `global_path_visible`
- `local_path_visible`
- `waypoint_visible`
- `active_goal_visible`
- `object_goal_visible`
- `elevated_target_visible`
- `mission_state_visible`
- `safety_state_visible`
- `camera_state_visible`
- `sound_state_visible`
- `ui_command_panel_alive`
- `ui_direct_cmd_vel_disabled`
- `cmd_vel_publishers`
- `overall_ui_success`

## Rosbag

Use `scripts/record_waver_experiment_bag.sh` or:

```bash
mkdir -p ~/ros2_ws/experiments_result/manual_rosbag
ros2 bag record /tf /tf_static /map /odom /plan /local_plan /cmd_vel \
  /waver/manual_cmd_vel /waver/cmd_vel_nav2 \
  /waver/lidar_objects /waver/lidar_objects_map /waver/elevated_dynamic_targets \
  /waver/mission_state /waver/safety_state \
  -o ~/ros2_ws/experiments_result/manual_rosbag/pre_real_validation
```

Do not commit `.db3`, `.mcap`, or `rosbag/` directories.

## Paper-Ready Export

After running H1/H2/H3 and UI checks, generate the cleaned paper package:

```bash
python3 ~/ros2_ws/src/FSD_Vehicle/src/waver_patrol/scripts/prepare_paper_results.py \
  --input-dir ~/ros2_ws/experiments_result \
  --output-root ~/ros2_ws/experiments_result/paper_ready
```

The script keeps all raw trials local, selects the latest row for each scenario
(`H1_elevated_dynamic`, `H2_elevated_static`, `H3_low_altitude_dynamic`) and each
UI trial, and writes paper metrics such as height filtering accuracy, target
precision/recall, false positive rates, UI success rate, and target mission
trigger success rate.
