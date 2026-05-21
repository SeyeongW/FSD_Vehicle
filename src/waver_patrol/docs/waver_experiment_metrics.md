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

## Radar Target Response

- Radar target to mission transition latency: `radar_targets.csv` + `mission_events.csv`
- Radar target to Nav2 goal latency: `radar_targets.csv` + `nav2_feedback.csv`
- Radar target to arrival latency: `target_mission_summary.csv`
- Radar confidence vs success: `radar_targets.csv.confidence` + `target_mission_summary.csv.navigation_success`
- Doppler/motion vs success: `radar_targets.csv.doppler_mps`

## Perception

- 3D LiDAR re-detection success: `lidar_targets.csv.accepted_as_aerial_target`
- Classification latency: `camera_classification.csv.classification_latency_ms`
- Bird confidence distribution: `camera_classification.csv.confidence`
- Unknown/false target ratio: `camera_classification.csv.target_class`

## Deterrence Task

- Bird confirmed to sound request latency: `camera_classification.csv` + `sound_events.csv`
- Sound task completion rate: `sound_events.csv.sound_task_done`
- Cooldown activations: `sound_events.csv.cooldown_active`
- Sound-to-patrol resume latency: `target_mission_summary.csv`

## Mission Recovery

- Interrupted waypoint return success: `target_mission_summary.csv.resumed_patrol`
- Return-to-patrol time: `target_mission_summary.csv.return_to_waypoint_*`
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

1. Robot XY trajectory with waypoint and radar target markers.
2. Mission state timeline.
3. Radar target response latency box plot.
4. Safety state duration stacked bar.
5. Battery voltage/percentage over mission time.
6. Classification confidence histogram.
7. Sound task request/completion timeline.
