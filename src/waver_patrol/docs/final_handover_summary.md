# Final Handover Summary

Active workspace: `~/ros2_ws`
Active repository: `~/ros2_ws/src/FSD_Vehicle`
Branch: `jo`

## What This Version Provides

- Existing `ugv_slam` mapping launch files remain the mapping entry point.
- Existing `ugv_nav`/Nav2 launch files remain the localization/navigation entry point.
- `waver_patrol` adds Waver mission, safety mux, target transform/filter, sound/camera stubs, Gazebo validation, and CSV/logging.
- `ugv_tools` operator panel now displays map, pose, yaw, global/local paths, current waypoint, goals, cluster candidates, elevated dynamic targets, and mission/safety/camera/sound states.
- Final `/cmd_vel` is owned by `safety_cmd_mux_node`; UI and perception nodes do not publish final `/cmd_vel`.

## Target Definition

The target is **not** an object that moved 3 m. The target is:

```text
object_height_m >= target_min_height_m
z_valid == true
dynamic_filter_pass == true
ego_motion_compensated == true
```

Default `target_min_height_m` is `3.0`.

## Perception Path

```text
3D PointCloud2 or Gazebo 3D PoseArray
  -> /waver/lidar_objects
  -> moving_object_map_transform_node
  -> /waver/lidar_objects_map
  -> moving_object_motion_filter_node
  -> /waver/elevated_dynamic_targets
  -> target_goal_manager_node
  -> /waver/object_mission_goal
  -> mission_patrol_manager_node
```

Active repo does not include `pcd_cluster_pkg`. Archive copy exists at `~/ros2_ws/FSD_Vehicle/src/ugv_main/pcd_cluster_pkg`. The active real-robot substitute is `waver_patrol/perception/pointcloud_lidar_objects_node.py`, which converts PointCloud2 clusters into `/waver/lidar_objects` without publishing velocity commands.

## Gazebo Validation Evidence

Last height-based isolated run:

```text
~/ros2_ws/experiments_result/
```

Summary:

- H1 elevated dynamic z=3.2 m: success, valid target.
- H2 elevated static z=3.2 m: success, rejected.
- H3 low dynamic z=1.0 m: success, rejected.
- UI U1 `slam_live` label/topic check: success.
- UI U3 command-button check: success; command topics were seen and direct `/cmd_vel` remained disabled.

## Build Status

Focused and full workspace builds passed:

```bash
colcon build --packages-select ugv_tools waver_patrol --symlink-install
colcon build --symlink-install
```

`livox_ros_driver2` was fixed to select the Humble rosidl branch automatically when
`ROS_DISTRO=humble`, so plain workspace builds no longer need a manual
`-DHUMBLE_ROS=humble` flag.

These are simulation gates only. Real deployment still requires live 3D z source, TF, localization, scan safety, wheel-off motion direction tests, and single `/cmd_vel` ownership.

## Run Commands

Focused build:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_tools waver_patrol --symlink-install
source install/setup.bash
```

Gazebo rover only:

```bash
ros2 launch waver_patrol waver_gazebo_rover_only.launch.py use_gui:=true
```

Operator panel:

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  require_scan:=false \
  auto_mode_strategy:=mission_nav2 \
  map_display_mode:=auto
```

Height H1/H2/H3 validation:

```bash
OUTPUT_ROOT=$HOME/ros2_ws/experiments_result \
TRIALS="1 2 3" \
REQUIRED_SUCCESSES=3 \
RECORD_BAG=false \
TARGET_MIN_HEIGHT_M=3.0 \
MIN_DYNAMIC_MOTION_M=0.2 \
MIN_DYNAMIC_VELOCITY_MPS=0.05 \
bash ~/ros2_ws/src/FSD_Vehicle/src/waver_patrol/scripts/run_pre_real_gazebo_trials.sh
```

UI validation:

```bash
python3 ~/ros2_ws/src/FSD_Vehicle/src/waver_patrol/scripts/run_ui_visualization_check.py \
  --output-root ~/ros2_ws/experiments_result \
  --trial-id U3 \
  --map-mode map_fixed \
  --require-command
```

Preflight:

```bash
bash ~/ros2_ws/src/FSD_Vehicle/src/waver_patrol/scripts/waver_real_preflight_check.sh
```

## Remaining Manual Checks

- Confirm actual 3D LiDAR topic and frame on Jetson.
- Confirm `map -> odom -> base_link -> lidar_frame` TF.
- Confirm `/cmd_vel` publisher is one and named `safety_cmd_mux_node`.
- Confirm operator panel shows `MAP_FIXED` in saved-map patrol and the map does not rotate with Waver yaw.
- Confirm sound output remains disabled until legal/safety approval.
