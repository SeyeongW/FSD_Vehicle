# Waver Experiment Data

`experiment_data_logger_node` creates one run folder per experiment under
`~/ros2_ws/waver_experiments/<timestamp>_<experiment_name>/`.

Each run contains:

- `metadata.yaml`: robot, ROS, map, operator notes, and safety configuration.
- `mission_events.csv`: mission state transitions and reasons.
- `robot_pose.csv`: odom/AMCL pose and motion samples.
- `cmd_vel.csv`: Nav2 candidate and final safety output velocity samples.
- `radar_targets.csv`: 4D radar goal candidates and acceptance metadata.
- `lidar_targets.csv`: 3D LiDAR/aerial target candidates.
- `camera_classification.csv`: external classifier output normalized for Waver.
- `sound_events.csv`: simulated/approved-speaker task status.
- `battery.csv`: percentage/voltage and return-home state.
- `obstacle_metrics.csv`: scan-derived obstacle metrics.
- `waypoint_progress.csv`: patrol progress and interruption/resume markers.
- `target_mission_summary.csv`: one row per radar/object mission.
- `experiment_summary.csv`: one row per run.

Raw replayability should be preserved with rosbag2. CSV files are for paper
tables and plots; rosbag2 is the raw evidence trail.
