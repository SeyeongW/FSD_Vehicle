# UGV Tools Operator Map UI

The active operator panel is:

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  require_scan:=false \
  auto_mode_strategy:=mission_nav2 \
  map_topic:=/map \
  map_display_mode:=auto \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan
```

## What It Shows

- `/map` as a 2D `nav_msgs/msg/OccupancyGrid`.
- Robot pose from `/amcl_pose` first, `/odom` fallback.
- Robot yaw as a red heading arrow. The map image itself is never rotated by robot yaw.
- Nav2 global path from `/plan`.
- Nav2 local path from `/local_plan`.
- Active Nav2/mission goal from `/waver/active_nav_goal`.
- Object mission goal from `/waver/object_mission_goal`.
- Current waypoint from `/waver/current_waypoint`.
- Cluster candidates from `/waver/lidar_objects_map`.
- Valid elevated dynamic targets from `/waver/elevated_dynamic_targets`.
- Mission, patrol, safety, height-filter, camera, sound, and Gazebo trial states.

## Control Safety

Default mode is safe for the real robot:

- publishes manual candidate velocity only to `/waver/manual_cmd_vel`,
- publishes mode commands to `/waver/mode`,
- publishes E-stop state to `/waver/emergency_stop`,
- does **not** publish final `/cmd_vel` unless `publish_direct_cmd_vel:=true` is explicitly set for legacy tests.

Final `/cmd_vel` must have one publisher:

```bash
ros2 topic info -v /cmd_vel
```

Expected publisher: `safety_cmd_mux_node`.

## Fixed Map Behavior

The panel treats `map` and `odom` data as fixed-world coordinates. If local path or candidate objects arrive in `base_link`, `base_footprint`, `laser`, `lidar`, or `camera_link`, the panel projects them through the current robot pose before drawing. This prevents the display from making static obstacles appear to orbit the robot during yaw rotation.

Use:

```bash
map_display_mode:=slam_live
```

for Mapping Mode labels, and:

```bash
map_display_mode:=map_fixed
```

for saved-map Patrol Mode labels.

## Troubleshooting

- `NO MAP`: confirm `/map` exists and the panel `map_topic` matches.
- `NO GLOBAL PATH`: check actual Nav2 path topic with `ros2 topic list | grep plan`.
- `NO POSE`: confirm `/amcl_pose` or `/odom` is publishing.
- target not shown: check `/waver/elevated_dynamic_targets` and `/waver/lidar_objects_map`.
- map appears to rotate: verify that RViz/operator fixed frame is `map`, and that object filters use `/waver/lidar_objects_map`, not raw sensor-frame displacement.
