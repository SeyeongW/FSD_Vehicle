# Moving Object Map Transform

This feature converts raw LiDAR object coordinates into the common SLAM frame
used by Waver navigation.

## Purpose

`/waver/lidar_objects`, `/lidar/detections`, or other moving-object topics may be
published in a LiDAR/sensor frame. That is useful for perception, but the robot
cannot safely use raw sensor coordinates as a shared mission reference while it
moves. `moving_object_map_transform_node` uses tf2 to transform those coordinates
into `map` or, if `map` is unavailable, `odom`.

The node only publishes data and RViz markers. It never publishes `/cmd_vel` and
does not override 4D radar mission topics.

## Default Topics

Input:

- `/waver/lidar_objects` as `geometry_msgs/msg/PoseArray`
- Alternative by parameter: `PointStamped` or `MarkerArray`

Output:

- `/waver/lidar_objects_map` as `geometry_msgs/msg/PoseArray`
- `/waver/lidar_object_point_map` as `geometry_msgs/msg/PointStamped`
- `/waver/moving_objects_map_marker` as `visualization_msgs/msg/MarkerArray`
- `/waver/moving_object_map_transform_state` as `std_msgs/msg/String`

Validation-only goal preview:

- `/waver/lidar_object_goal_preview` as `geometry_msgs/msg/PoseStamped`
- `/waver/lidar_object_goal_preview_marker`

The preview goal is not a Nav2 command and is not connected to
`/waver/object_mission_goal` by default. This keeps onboard LiDAR data separate
from 4D radar mission commands.

## Required TF Tree

At least one transform path must exist:

```text
map -> odom -> base_link -> <lidar_frame>
```

or, for odom-only tests:

```text
odom -> base_link -> <lidar_frame>
```

If `map` lookup fails, the node tries `odom` when `fallback_frame=odom`.

## Run

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch waver_patrol moving_object_map_transform.launch.py \
  input_type:=pose_array \
  input_topic:=/waver/lidar_objects \
  target_frame:=map \
  fallback_frame:=odom
```

Odom-only test:

```bash
ros2 launch waver_patrol moving_object_map_transform.launch.py \
  input_type:=pose_array \
  input_topic:=/waver/lidar_objects \
  target_frame:=odom \
  fallback_frame:=
```

Mission launch integration, still data-only by default:

```bash
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  enable_pointcloud_lidar_objects:=true \
  enable_moving_object_map_transform:=true \
  pointcloud_topic:=/mid360_PointCloud2 \
  require_scan:=true \
  start_serial_bridge:=false
```

Enable validation preview goal only when testing:

```bash
ros2 launch waver_patrol moving_object_map_transform.launch.py \
  enable_goal_relay:=true
```

## RViz

1. Start RViz.
2. Set Fixed Frame to `map` or `odom`.
3. Add `MarkerArray`.
4. Select `/waver/moving_objects_map_marker`.
5. Confirm object markers stay in the same map location while the robot moves.

## Checks

```bash
ros2 topic echo /waver/lidar_objects_map
ros2 topic echo /waver/moving_object_map_transform_state
ros2 topic echo /waver/moving_objects_map_marker
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map <lidar_frame>
```

`header.frame_id` on transformed outputs must be `map` or `odom`.

## Command Separation

4D radar mission commands:

```text
/radar4d/* -> /waver/radar_target_goal -> /waver/object_mission_goal
```

Waver onboard LiDAR common-frame data:

```text
/waver/lidar_objects -> /waver/lidar_objects_map
```

These remain separate unless an operator explicitly connects the preview goal to
mission logic. This prevents LiDAR detections and 4D radar instructions from
competing for robot command authority.
