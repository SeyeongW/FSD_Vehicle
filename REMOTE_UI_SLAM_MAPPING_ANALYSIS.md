# Remote UI SLAM Mapping Analysis

Scope: `~/ros2_ws3/FSD_Vehicle` only.

Protected baseline: `~/ros2_ws2` was backed up before this work and was not
modified.

## Existing Findings

- The remote panel already publishes manual commands on `/waver/manual_cmd_vel`
  and keeps `publish_direct_cmd_vel` disabled by default.
- The mapping workflow manager already supports `START_MAPPING`, `SAVE_MAP`,
  and `APPLY_FIXED_MAP`, but older defaults pointed at `~/ros2_ws2`.
- `ugv_gazebo` already provides `/scan` and `/odom` through the Gazebo rover SDF.
- Existing bird-patrol launch files mix mapping, mission, and tracking concerns;
  the new UI-SLAM launch isolates mapping first.

## Improvement Strategy

- Add a Gazebo-specific launch under `ugv_gazebo/launch/ui_slam`.
- Mirror Gazebo `/scan` into `/scan_slam` and `/scan_safety`.
- Run actual `slam_toolbox` as the default `/map` authority.
- Keep final `/cmd_vel` authority in `safety_cmd_mux_node`.
- Keep `/waver/mode` authority in `mission_patrol_manager_node`.
- Use `waver_gazebo_patrol` for odom waypoint patrol after mapping.

## New Patrol Route

`src/waver_patrol/waypoints/waver_4m_then_7m_square_patrol.yaml`

The route is interpreted relative to the robot start pose:

1. 4 m straight leg.
2. 7 m square patrol.

## Added Runtime Audit

`mapping_backend_manager_node` publishes:

- `/waver/mapping_backend_state`
- `/waver/mapping_backend_ready`
- `/waver/mapping_backend_fault`

It checks `/scan_slam`, `/odom`, `/map`, and TF health during actual mapping.
