# Waver Real-Vehicle Release Notes

This branch is prepared as a pre-real validation baseline, not as an automatic
wheel-on command. Keep the robot lifted or drive wheels disabled until the
preflight checks pass.

## Command Path

- Operator UI WASD/arrow keys publish only `/waver/manual_cmd_vel`.
- Nav2 output must be remapped to `/waver/cmd_vel_nav2`.
- `safety_cmd_mux_node` is the only allowed final `/cmd_vel` publisher.
- Legacy `ugv_driver` is disabled by default. Use
  `legacy_driver_enabled:=true start_driver:=true` only for archived legacy tests.
- The canonical serial bridge must subscribe only to final `/cmd_vel`.

## Real Backend

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=false \
  include_existing_ugv_driver:=false \
  remap_nav2_cmd_vel:=true \
  enable_pointcloud_lidar_objects:=true \
  enable_moving_object_map_transform:=true \
  enable_moving_object_motion_filter:=true \
  enable_deep_learning_stub:=false \
  enable_sound_stub:=false
```

## Operator UI

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  require_scan:=false \
  publish_direct_cmd_vel:=false \
  profile:=real
```

WASD and arrow keys remain enabled. They are manual candidates only and must pass
through the safety mux before any motor command is written.

## SLAM Mapping Workflow

1. Press `SLAM MAPPING` in the UI. The old map display and overlays are cleared,
   but existing map files are not deleted.
2. The panel enters `MAPPING_AUTO` and starts
   `waver_mapping_backend.launch.py backend:=cartographer` by default.
3. The live `/map` is rendered as `SLAM_LIVE`.
4. Press `SAVE MAP`. The mapping workflow manager saves
   `~/ros2_ws/maps/waver_latest_map.yaml` and keeps it unapplied.
5. Press `APPLY FIXED MAP`. The UI state becomes `MAP_FIXED_READY`; start the
   localization/Nav2 backend for the saved map before wheel-on patrol.

Simulation-only workflow validation can use:

```bash
ros2 launch waver_patrol waver_mapping_backend.launch.py backend:=gazebo_live use_rviz:=false
```

## Patrol And Object Mission

- `START PATROL` is allowed only after a fixed map, localization, safety, and
  route are ready.
- Height-based target condition is:
  `object_height_m >= 3.0`, `z_valid=true`, `dynamic_filter_pass=true`, and
  ego-motion compensation completed in map/odom.
- Target approach goals are generated from the current robot pose toward the
  object with a safety offset, not from the map origin.
- Mapping mode disables object target interrupts by default.

## E-Stop

- `STOP` cancels motion and returns to `STANDBY`.
- `E-STOP` latches `EMERGENCY`; reset is required before driving.
- Scan stale, TF/localization stale, duplicate final `/cmd_vel`, serial fault, or
  E-stop must hold zero command.

## Wheel-Off Test

1. Verify Python compile and focused colcon build.
2. Launch backend with `start_serial_bridge:=false`.
3. Launch UI with `publish_direct_cmd_vel:=false`.
4. Confirm `/cmd_vel` has exactly one publisher and `ugv_driver` is absent.
5. Press each UI command and verify only candidate/safety topics change.

## Wheel-On Low-Speed Test

After wheel-off tests pass, enable the canonical serial bridge only. Start at
0.10 to 0.15 m/s maximum linear speed, keep E-stop in hand, and test manual
WASD before AUTO/PATROL.

## Known Limitations

- Height filtering requires real z data from 3D LiDAR, depth, stereo, or a
  custom 3D detection source. 2D LaserScan cannot prove height>=3 m.
- The operator panel can start a mapping backend, but full production map-server
  lifecycle orchestration should be checked on the robot PC before wheel-on.
- Sound and classifier stubs are disabled by default for real profile; use real
  reviewed backends or explicit dry-run mode.
