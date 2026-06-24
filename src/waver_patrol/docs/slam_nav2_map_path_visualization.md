# SLAM, Nav2, Map, And Path Visualization

## Mapping Mode

Purpose:

1. start LiDAR/odom/TF,
2. run `ugv_slam` or Waver mapping wrapper,
3. create live `/map`,
4. show `SLAM_LIVE` in the operator panel/RViz,
5. save the map before Patrol Mode.

Commands:

```bash
cd ~/ugv_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch waver_patrol waver_mapping_2d.launch.py \
  algorithm:=cartographer \
  use_rviz:=true
```

or:

```bash
ros2 launch ugv_slam cartographer.launch.py
ros2 launch ugv_slam gmapping.launch.py
```

Operator panel:

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_display_mode:=slam_live \
  map_topic:=/map \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  require_scan:=true \
  auto_mode_strategy:=mission_nav2
```

Mapping Mode must not trigger target navigation. Elevated target candidates may be logged for debugging, but mission triggers should stay disabled until the map is saved.

## Map Save

```bash
mkdir -p ~/ugv_ws/FSD_Vehicle/maps
ros2 run nav2_map_server map_saver_cli -f ~/ugv_ws/FSD_Vehicle/maps/patrol_map
```

## Saved-Map Patrol Mode

Purpose:

1. load `patrol_map.yaml`,
2. run localization/AMCL,
3. run Nav2,
4. follow waypoints,
5. keep the map fixed while robot pose/path/targets move.

Commands:

```bash
ros2 launch waver_patrol waver_localization.launch.py \
  map:=~/ugv_ws/FSD_Vehicle/maps/patrol_map.yaml \
  use_rviz:=true
```

or existing Nav2 wrapper:

```bash
ros2 launch ugv_nav nav.launch.py \
  use_localization:=amcl \
  use_localplan:=teb \
  map:=~/ugv_ws/FSD_Vehicle/maps/patrol_map.yaml
```

Operator panel:

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_display_mode:=map_fixed \
  map_topic:=/map \
  amcl_pose_topic:=/amcl_pose \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  elevated_dynamic_target_topic:=/waver/elevated_dynamic_targets \
  require_scan:=true \
  auto_mode_strategy:=mission_nav2
```

## Required TF Tree

```text
map
  -> odom
      -> base_link
          -> lidar_frame
          -> camera_link
```

Check:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link lidar_frame
```

## Path Topic Check

Nav2 deployments use different path names. Verify actual topics before field tests:

```bash
ros2 topic list | grep -E 'plan|path|costmap'
ros2 topic info /plan
ros2 topic info /local_plan
```

If your Nav2 stack uses `/global_plan` or `/received_global_plan`, pass it to the panel with `global_path_topic:=...`.
