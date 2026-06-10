# ROS2 WS3 Gazebo Bird Patrol Integration

Workspace: `/home/chotaehyun/ros2_ws3/FSD_Vehicle`

Branch: `jo`

This workspace is the Gazebo integration sandbox for the Waver bird patrol mechanism. The active code branch is `jo`. The `seo` branch may be checked out only in a separate read-only worktree for Gazebo/RViz/topic-structure reference.

## Scope

The implemented Gazebo mechanism is:

1. Launch Gazebo Classic with Waver UGV, Mid360/Livox point cloud, camera topics, and Gazebo bird target.
2. Launch `waver_remote_panel` as the operator UI.
3. Start patrol through `/waver/mission_command` or the UI Start Patrol path.
4. Patrol starts with a 4 m straight segment, then repeats the paper square waypoint pattern.
5. Gazebo LiDAR/target tracker publishes dynamic target lock topics.
6. Patrol is interrupted and a target-relative inspection offset goal is generated.
7. The robot approaches the target offset, aligns camera/body, performs Gazebo fake bird classification, triggers the simulated sound stub, returns to the interrupted waypoint, and resumes patrol.

## Build

```bash
cd ~/ros2_ws3/FSD_Vehicle
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  ugv_description ugv_gazebo ugv_tools \
  waver_patrol waver_seo_tracking waver_experiment_logger \
  livox_ros_driver2 ros2_livox_simulation

source install/setup.bash
test -f install/ros2_livox_simulation/lib/libros2_livox.so
```

## Verified Entrypoints

Headless smoke run:

```bash
cd ~/ros2_ws3/FSD_Vehicle
USE_GUI=false ENABLE_RVIZ=false START_REMOTE_PANEL=true SKIP_BUILD=true TIMEOUT_SEC=260 RANDOM_SEED=530 \
  bash scripts/run_gazebo_lidar_spatial_visual_experiment.sh
```

Gazebo GUI and RViz:

```bash
cd ~/ros2_ws3/FSD_Vehicle
USE_GUI=true ENABLE_RVIZ=true START_REMOTE_PANEL=true SKIP_BUILD=true TIMEOUT_SEC=260 RANDOM_SEED=530 \
  bash scripts/run_gazebo_lidar_spatial_visual_experiment.sh
```

The final launch file is wrapped by the runner and provided by `ugv_gazebo`:

```text
src/ugv_main/ugv_gazebo/launch/bird_patrol/ugv_gazebo_bird_patrol_seo.launch.py
```

## Selected Gazebo Assets

- World: `src/ugv_main/ugv_gazebo/worlds/ugv_world.world`
- Robot model: `src/ugv_main/ugv_gazebo/models/ugv_rover/model.sdf`
- Bird model: `src/ugv_main/ugv_gazebo/models/bird/model.sdf`
- Bird runtime manager: `src/ugv_main/ugv_gazebo/scripts/bird_manager.py`
- Mid360 CSV: `src/livox_laser_simulation_RO2/scan_mode/mid360.csv`

## Key Topics And Authority

- Final `/cmd_vel` publisher: `safety_cmd_mux_node`
- `/waver/mode` publisher: `mission_patrol_manager_node`
- Nav/patrol candidate command: `/waver/cmd_vel_nav2`
- Target/body tracking candidate command: `/waver/cmd_vel_target_track`
- Selected auto command: `/waver/cmd_vel_auto_selected`
- Final robot command: `/cmd_vel`
- Dynamic target array: `/waver/elevated_dynamic_targets`
- Target lock: `/waver/dynamic_object_lock`
- Target lock state: `/waver/dynamic_object_lock_state`
- Classification state: `/waver/classification_state`
- Sound state: `/waver/sound_alert_state`
- Trial logger state: `/waver/seo_trial_logger_state`
- Spatial debug markers: `/waver/spatial_debug_markers`

The final `/cmd_vel` authority must remain:

```text
simple_nav2_cmd_sim_node or target/body command source
  -> intermediate command topic
  -> mission_state_cmd_selector_node
  -> safety_cmd_mux_node
  -> /cmd_vel
```

## Verification Commands

```bash
cd ~/ros2_ws3/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_DOMAIN_ID

ros2 topic info -v /cmd_vel
ros2 topic info -v /waver/mode
ros2 topic echo --once /waver/seo_trial_logger_state
ros2 topic echo --once /waver/mission_state
ros2 topic echo --once /waver/dynamic_object_lock_state
```

The runner stores a graph snapshot during launch:

```text
<run_dir>/logs/cmd_vel_topic_info.txt
<run_dir>/logs/topic_list_snapshot.txt
<run_dir>/logs/node_list_snapshot.txt
```

Verifier:

```bash
python3 scripts/verify_gazebo_spatial_response_trial.py <run_dir> --mode spatial --min-removed-birds 2
python3 scripts/verify_gazebo_spatial_response_trial.py <run_dir> --mode full --min-removed-birds 2
```

Expected result:

```text
VERIFY_GAZEBO_SPATIAL_RESPONSE=PASS
GAZEBO_LIDAR_SPATIAL_VISUAL_EXPERIMENT=PASS
```

## Output Policy

Runtime outputs are generated under:

```text
experiment_results/
```

They are intentionally ignored by Git. Use the generated `metrics/` and `logs/` for paper analysis, but do not commit generated runs.

## Paper Metrics

Core output files:

```text
logs/spatial_distance_timeseries.csv
logs/goal_bird_distance_events.csv
logs/bird_kinematics.csv
logs/lidar_filter_response.csv
logs/lidar_waver_latency_events.csv
logs/cmd_vel_response.csv
metrics/spatial_response_metrics.json
metrics/paper_spatial_table.csv
metrics/paper_latency_table.csv
```

Important clean metrics:

```text
spatial.target_goal_to_bird_xy_first_m
spatial.target_goal_to_lidar_target_xy_mean_m
spatial.valid_lidar_target_to_bird_xy_mean_m
lidar.filter_runtime_wall_mean_ms
latency.active_target_nav_goal_to_cmd_vel_ms
latency.active_target_nav_goal_to_odom_motion_ms
navigation.odom_path_length_m
safety.cmd_vel_safety_mux_sole_publisher
mechanism.lidar_only_decision_clean
```

## Target Pose Smoothing

The Gazebo target tracker keeps target coordinates continuous by preferring the Gazebo bird pose fallback during short windows, applying output smoothing, and holding the last valid pose during short detection dropouts.

Configured in:

```text
src/ugv_main/ugv_gazebo/param/bird_patrol/seo_tracking_gazebo.yaml
```

Important parameters:

```yaml
hold_last_target_sec: 1.0
output_smoothing_alpha: 0.30
prefer_gazebo_fallback_sec: 0.8
```

## Real Prep Files

The following files prepare real-vehicle remap/profile structure only. They are not a real launch and must not be treated as field validation.

```text
src/ugv_main/ugv_gazebo/param/bird_patrol/profiles/real_prep.yaml
src/ugv_main/ugv_gazebo/param/bird_patrol/topic_remaps_real_prep.yaml
```

Real-prep topic notes include:

```text
/mid360_PointCloud2 -> /livox/lider candidate remap
/camera/image_raw
/camera/camera_info
/waver/manual_cmd_vel
/waver/mode_cmd
/waver/mission_command
/cmd_vel final through safety_cmd_mux_node
```

## Safety Notes

- The Gazebo classifier is fake/demo classification: `GAZEBO_FAKE_YOLO11S_CLASSIFICATION`.
- This does not prove real bird detection precision, recall, F1, or mAP.
- Real vehicle execution is explicitly out of scope for this Gazebo integration task.
- Real vehicle testing still requires actual camera-LiDAR calibration, Mid360 frame validation, physical E-stop, battery validation, and real detector model verification.
