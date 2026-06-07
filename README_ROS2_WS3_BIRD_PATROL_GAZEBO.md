# ROS2 WS3 Gazebo Bird Patrol Integration

Workspace: `/home/chotaehyun/ros2_ws3/FSD_Vehicle`

Branch: `jo`

This workspace is the Gazebo integration sandbox for the Waver bird patrol mechanism. The local `ros2_ws2` workspace and the cloned `seo` branch archive are reference-only sources and must not be modified by this integration.

## Scope

The implemented Gazebo mechanism is:

1. Launch Gazebo Classic with Waver UGV, Mid360/Livox point cloud, camera topics, and Gazebo bird target.
2. Launch `waver_remote_panel` as the operator UI.
3. Start patrol through `/waver/mission_command` or the UI Start Patrol path.
4. Patrol starts with a 4 m straight segment, then repeats a 7 m square waypoint pattern.
5. Gazebo LiDAR/target tracker publishes dynamic target lock topics.
6. Patrol is interrupted and a target-relative inspection offset goal is generated.
7. The robot approaches the target offset, aligns camera/body, performs Gazebo fake bird classification, triggers the simulated sound stub, returns to the interrupted waypoint, and resumes patrol.

## Final Gazebo Entrypoint

```bash
cd ~/ros2_ws3/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_DOMAIN_ID

ros2 launch ugv_gazebo ugv_gazebo_bird_patrol_seo.launch.py \
  use_gui:=true \
  start_remote_panel:=true \
  enable_trial_logger:=true \
  remote_panel_demo_script:=start_patrol_once
```

The final launch file is provided by `ugv_gazebo`:

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

A successful mechanism run should show:

```text
SEEN=patrol,dynamic_target,lock,inspection_goal,camera_align,classification,sound,return_or_resume MISSING=none
```

## Latest Verified Run

The mechanism was directly verified with Gazebo GUI and `waver_remote_panel` running. The latest verified successful log set is:

```text
experiments_result/gazebo_bird_patrol/gazebo_seo_bird_patrol_20260603_072230/mechanism_events.csv
experiments_result/gazebo_bird_patrol/gazebo_trial_01_20260603_072230/
```

Observed mission flow:

```text
APPROACH_TARGET_OFFSET
TARGET_REACHED
CAMERA_ALIGN_DONE
TARGET_CLASSIFICATION_WAIT
SOUND_TASK_REQUESTED
SOUND_TASK_RUNNING
SOUND_TASK_DONE
RETURN_TO_INTERRUPTED_WAYPOINT
RESUME_PATROL
PATROL_NAVIGATING
```


## Mechanism Log Verification

The latest verified log can be checked without relaunching Gazebo:

```bash
cd ~/ros2_ws3/FSD_Vehicle
python3 scripts/verify_ros2_ws3_bird_patrol_log.py \
  experiments_result/gazebo_bird_patrol/gazebo_seo_bird_patrol_20260603_072230/mechanism_events.csv
```

Expected result from the latest run:

```text
MECHANISM_LOG_VERIFY=PASS rows=4358 nonzero_cmd_rows=3994 odom_dx=5.948 odom_dy=0.127
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
