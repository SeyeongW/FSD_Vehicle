# Current Waver Mechanism Analysis

## Flow Diagram

```text
SLAM Mapping
  -> Save Map
  -> Localization / Nav2 / AMCL or odom fallback
  -> Waypoint Patrol
  -> ugv_tools Fixed Map UI
  -> LiDAR / PointCloud Cluster Candidate
  -> tf2 Map/Odom Transform
  -> Elevated Dynamic Filter (height >= 3 m + map/odom motion)
  -> Pause Patrol
  -> Navigate To Target Safety Offset
  -> Yaw Alignment
  -> Camera Detection / Classification Stub
  -> Sound Mission Stub
  -> Mission Complete
  -> Resume Saved Waypoint Patrol
  -> experiments_result Logging
```

## Command Path

```text
mission_patrol_manager / Nav2 candidate
  -> /waver/cmd_vel_nav2

operator panel / keyboard
  -> /waver/manual_cmd_vel

safety_cmd_mux_node
  inputs: /waver/cmd_vel_nav2, /waver/manual_cmd_vel, /scan, /waver/mode,
          /waver/emergency_stop, /waver/external_stop, /waver/speed_limit
  output: /cmd_vel

Gazebo skid_steer_drive_controller or real serial bridge
  subscribes: /cmd_vel
```

Final `/cmd_vel` must have exactly one publisher: `safety_cmd_mux_node`.

## Gazebo Mechanism

- `ugv_gazebo/worlds/ugv_world.world` is used as the base world.
- `ugv_rover` is spawned from `ugv_gazebo/models/ugv_rover/model.sdf`.
- `gazebo_moving_object_trial_publisher_node` publishes simulated cluster centers on `/waver/lidar_objects`.
- `moving_object_map_transform_node` transforms those object candidates into `/waver/lidar_objects_map`.
- `moving_object_motion_filter_node` confirms `height>=3.0 m`, `z_valid=true`, and small map/odom compensated dynamic motion before publishing `/waver/elevated_dynamic_targets`.
- `mission_patrol_manager_node` pauses patrol, sends target mission, performs simulated arrival/yaw/camera/sound states, then resumes patrol.
- `gazebo_trial_logger_node` writes mission, object, navigation, yaw, camera, sound, safety, and summary CSV files.

## Remote Panel Mechanism

`ugv_tools/waver_remote_panel.py` is a visual remote. It publishes manual candidate commands, mission commands, mode, E-stop, and speed-limit topics. It now renders `/map`, `/amcl_pose` or `/odom`, `/plan`, `/local_plan`, current waypoint, active goals, cluster candidates, and `/waver/elevated_dynamic_targets` in a fixed map/odom canvas. If a local planner path or object candidate arrives in a robot-relative frame such as `base_link`, the panel projects it into the current odom plane so the map does not rotate with the vehicle.

The panel defaults to `publish_direct_cmd_vel=false`, so it publishes manual motion only on `/waver/manual_cmd_vel`. The final `/cmd_vel` remains owned by `safety_cmd_mux_node`.

## Data Path

All new validation scripts default to:

```text
~/ros2_ws/experiments_result
```

`waver_experiments*` is treated as a legacy output path and should not be used
for final pre-real reports.

## Sound and Camera

Camera classification and sound output are stubbed for Gazebo validation. `enable_sound_output` remains false by default. The sound task is a non-contact warning/deterrent state only; no weapon, projectile, or harmful actuator control is implemented.
