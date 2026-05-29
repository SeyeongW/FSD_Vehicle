# Waver Experiment Metrics

This document maps Waver CSV logs to paper-ready tables and plots.

## Navigation

- Waypoint arrival success rate: `waypoint_progress.csv`, `mission_events.csv`
- Average waypoint travel time: `mission_events.csv`
- Total distance: `experiment_summary.csv.total_distance_m`
- Path efficiency: rosbag `/plan`, `/odom`, and `robot_pose.csv`
- Nav2 recovery count: `experiment_summary.csv.recovery_count`
- Replanning and failure count: `nav2_feedback.csv`, `mission_events.csv`
- Average speed/angular speed: `cmd_vel.csv`
- Safety hard stop count: `safety_state.csv`

## LiDAR-First Target Response

- Height >= 3 m dynamic target acceptance: `lidar_targets.csv.accepted_as_aerial_target`
- LiDAR target to inspection goal latency: `inspection_goals.csv.candidate_to_goal_latency_ms`
- Offset goal success: `inspection_goals.csv.goal_publish_success`
- Approach duration and final standoff: `target_approach.csv`
- Reject counts: `lidar_targets.csv.reject_reason`, `inspection_goals.csv.goal_reject_reason`
- Radar logs remain optional legacy context in `radar_targets.csv`.

## Perception

- 3D LiDAR re-detection success: `lidar_targets.csv.accepted_as_aerial_target`
- Camera/gimbal alignment success: `camera_alignment.csv.centered`
- Camera alignment latency and pointing error: `camera_alignment.csv`
- Classification latency: `camera_classification.csv.classification_latency_ms`
- Bird/drone/unknown/irrelevant distribution: `camera_classification.csv.target_class`

## Deterrence Task

- Bird confirmed to sound request latency: `camera_classification.csv` + `sound_events.csv`
- Sound task completion rate: `sound_events.csv.sound_task_done`
- Cooldown activations: `sound_events.csv.cooldown_active`
- Sound-to-patrol resume latency: `target_mission_summary.csv`

## Mission Recovery

- Interrupted waypoint return success: `target_mission_summary.csv.resumed_patrol`
- Return-to-patrol time: `return_to_patrol.csv`, `target_mission_summary.csv.return_to_waypoint_*`
- Battery interrupt count: `experiment_summary.csv.battery_return_count`

## Safety

- Scan stale stop count: `safety_state.csv.safety_state`
- Command timeout stop count: `safety_state.csv.safety_state`
- E-stop/external stop count: `experiment_summary.csv`
- Minimum obstacle distance: `obstacle_metrics.csv.min_scan_range_m`

## Battery

- Warning/critical time: `battery.csv`
- Return home success: `mission_events.csv`, `experiment_summary.csv`
- Voltage under load trend: `battery.csv.voltage`

## Recommended Figures

1. Robot XY trajectory with waypoint, LiDAR target, inspection goal, and classification point.
2. Mission state timeline.
3. LiDAR detection-to-goal and approach latency box plot.
4. Safety state duration stacked bar.
5. Battery voltage/percentage over mission time.
6. Classification confidence histogram.
7. Sound task request/completion timeline.

## Scripts

- `scripts/compute_paper_metrics.py <experiment_result_dir>` writes `paper_metrics_summary.csv/json`.
- `scripts/plot_paper_metrics.py <experiment_result_dir>` writes `plots/*.png`.
- `scripts/export_paper_tables.py <experiment_result_dir>` writes grouped CSV tables.
- Metrics that require manual labels or external ground truth are reported as `N/A - no ground truth`.
