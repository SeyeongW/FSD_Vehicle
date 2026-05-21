# Waver Patrol

Waver is a safety-first wrapper package for a Waveshare WAVE ROVER based outdoor patrol robot.
It does not overwrite the upstream Waveshare packages. It adds a new ROS2/Python package that
adapts keyboard teleop, `/cmd_vel`, Nav2, mapping, localization, and repeated waypoint patrol
through Waver-specific command sanitation, rate limiting, collision checks, and fail-stop rules.

## What This Extends

| Existing package/file | Reuse in Waver | Notes |
| --- | --- | --- |
| `ugv_bringup/bringup_lidar.launch.py` | optional include | Starts robot description, UGV bringup, driver, LiDAR, rf2o, base node. It may also own serial/cmd_vel. |
| `ugv_nav/nav.launch.py` | Nav2 wrapper | Uses AMCL/EMCL/Cartographer choices and existing Nav2 params. |
| `ugv_nav/slam_nav.launch.py` | SLAM+Nav wrapper | Supervised, low-speed only. Patrol should wait for reliable localization. |
| `ugv_slam/cartographer.launch.py` | 2D mapping wrapper | Preferred 2D mapping if available. |
| `ugv_slam/gmapping.launch.py` | 2D mapping fallback | Useful for simple 2D tests. |
| `ugv_slam/rtabmap_rgbd.launch.py` | 3D mapping wrapper | Requires RGB-D/OAK topics. |
| `ugv_tools/keyboard_ctrl.py` | reference only | Waver uses its own teleop so commands pass safety checks. |
| `ugv_bringup/ugv_driver.py` | reference only | Existing driver sends `T:13/X/Z`; Waver serial bridge sends `T:1/L/R`. Do not run both command paths on one serial port. |

## Safety Position

This package is not a claim of fully autonomous outdoor deployment. Outdoor patrol is allowed only
after wheel-off tests, low-speed manual driving, LiDAR health checks, map/localization validation,
and supervised Nav2 tests. A human E-Stop operator must be present around people, vehicles, bikes,
children, pets, or public spaces.

WAVE ROVER builds may use open-loop PWM without wheel encoders. In that case, `L/R` are PWM ratios,
not measured m/s. Without encoders, GPS/RTK, or reliable odometry, waypoint tracking, slip detection,
and true runaway detection are limited. Waver can still clamp commands and fail-stop on stale sensors,
but it cannot prove actual vehicle speed.

## Install And Build

This local workspace currently builds the Waver package from the FSD_Vehicle source tree:

```text
~/ros2_ws/src/FSD_Vehicle/src/waver_patrol
```

If another copy exists at `~/ros2_ws/src/waver_patrol`, keep only one build-visible copy. The
safe local option is to leave the files in place and add `COLCON_IGNORE` to the extra copy, instead
of deleting it. Duplicate `waver_patrol` packages will stop `colcon` before the robot can be tested.

If this zip restored `waver_patrol` only under the archived folder, recover it into the actual
colcon source space first:

```bash
cd /home/chotaehyun/ros2_ws
mkdir -p src
rsync -a _waver_archived_non_ugv_tools_20260519/src/waver_patrol/ src/FSD_Vehicle/src/waver_patrol/
```

Do not move or symlink the `FSD_Vehicle` trees while doing this; duplicate packages can break
`colcon`. Build `waver_patrol` alone first.

```bash
cd /home/chotaehyun/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install -i --from-paths src/FSD_Vehicle/src/waver_patrol --rosdistro humble -y
colcon build --packages-select waver_patrol --symlink-install
source install/setup.bash
```

In this workspace, `FSD_Vehicle` exists both at `~/ros2_ws/FSD_Vehicle` and
`~/ros2_ws/src/FSD_Vehicle`, so avoid a blind full-workspace build until the duplicate package paths
are intentionally handled. If `ugv_nav` or `ugv_slam` are not visible in `ros2 pkg list`, build the
needed upstream packages from one chosen source tree only.

## Stack Inspection

```bash
ros2 run waver_patrol inspect_stack
```

Real-robot preflight check, after starting the Waver launch but before enabling serial:

```bash
bash ~/ros2_ws/src/FSD_Vehicle/src/waver_patrol/scripts/waver_real_preflight_check.sh
```

The script does not publish motion commands. It verifies that exactly one build-visible
`waver_patrol` exists, checks `/cmd_vel` ownership, samples safety/sensor topics, and lists serial
process conflicts. If `/cmd_vel` has more than one publisher, do not run the robot.

## Stop

```bash
ros2 run waver_patrol stop_robot
```

`stop_robot` sends `{"T":1,"L":0,"R":0}` repeatedly. Stop commands ignore motor inversion and swap.

## Keyboard Teleop

```bash
ros2 run waver_patrol teleop_keyboard --ros-args -p config:=config/waver_keyboard.yaml
```

Keys: `W/I` forward, `S/,` reverse, `A/J` left, `D/L` right, diagonals `Q/U E/O Z/M C/.`,
`K/Space` immediate stop, `ESC/Ctrl+C` stop and exit, `+/-` speed, `[/]` turn gain,
`P` patrol pause/resume intent, `R` reset intent, `G` save current pose when ROS2 pose exists,
`N` skip, `B` return home, `H` help.

If no key arrives for 0.3 s, Waver stops.

## Mapping

```bash
ros2 launch waver_patrol waver_mapping_2d.launch.py algorithm:=cartographer use_rviz:=true
ros2 launch waver_patrol waver_mapping_2d.launch.py algorithm:=gmapping use_rviz:=true
ros2 launch waver_patrol waver_mapping_3d.launch.py use_rviz:=true
```

## Localization

```bash
ros2 launch waver_patrol waver_localization.launch.py map:=/path/to/map.yaml use_rviz:=true
```

## Navigation And Patrol

```bash
ros2 launch waver_patrol waver_nav2.launch.py use_localization:=amcl use_localplan:=dwa use_rviz:=true
ros2 launch waver_patrol waver_patrol.launch.py waypoints:=src/FSD_Vehicle/src/waver_patrol/waypoints/patrol_outdoor_demo.yaml use_rviz:=true
```

Important command-path rule: enable only one base-control path. If existing `ugv_nav` or
`ugv_bringup` starts `ugv_driver`, do not also start Waver serial bridge on the same serial device.
For Waver's safety path, route Nav2/manual/autonomy commands through Waver's mux/safety/bridge.

## Bird Autonomy Stack

The bird-autonomy launch keeps final velocity ownership simple: only `safety_cmd_mux_node`
publishes `/cmd_vel`. Patrol, moving target tracking, and return-home controllers publish candidate
commands under `/waver/*`; the auto mux selects one behavior, then the safety mux applies E-stop,
manual override, `/scan` hard-stop/slow-down, timeouts, speed limits, and rate limits.

```text
fixed_waypoint_patrol_node       -> /waver/cmd_vel_patrol
lidar_aerial_motion_detector_node -> /waver/aerial_target, /waver/aerial_target_active
target_body_tracker_node         -> /waver/cmd_vel_target_track
battery_return_manager_node      -> /waver/return_home_active, /waver/return_goal
return_home_controller_node      -> /waver/cmd_vel_return_home
auto_behavior_mux_node           -> /waver/cmd_vel_auto
safety_cmd_mux_node              -> /cmd_vel
serial_cmd_vel_bridge            <- /cmd_vel
```

`/scan` is a 2D `LaserScan`; it has no z value. Waver therefore uses `/scan` only for ground
obstacle stop/slow-down. Aerial or height-based target candidates must arrive as `PoseArray`,
`PointStamped`, or a `PointCloud2` adapter on `/waver/lidar_objects`, `/lidar/detections`,
or `/waver/object_point`.

`pcd_cluster_pkg/cluster_node.py` must not publish `/cmd_vel` on the real robot. In this workspace
it has been changed so direct command output is disabled by default and cluster centers are
published as `/waver/lidar_objects`.

Never run `ugv_driver` and `serial_cmd_vel_bridge` on the same serial port. If existing
`ugv_bringup` owns serial, keep `start_serial_bridge:=false`.

Desk test, with fake odom and fake targets only:

```bash
cd /home/chotaehyun/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch waver_patrol waver_bird_autonomy.launch.py \
  use_sim_odom:=true \
  enable_test_publishers:=true \
  require_scan:=false \
  start_serial_bridge:=false \
  default_mode:=AUTO
```

`require_scan:=false` is only for desk tests. Do not use it for real driving.

Pre-drive real robot test without serial output:

```bash
ros2 launch waver_patrol waver_bird_autonomy.launch.py \
  require_scan:=true \
  start_serial_bridge:=false \
  enable_test_publishers:=false \
  default_mode:=AUTO
```

Real serial bridge mode:

```bash
ros2 launch waver_patrol waver_bird_autonomy.launch.py \
  require_scan:=true \
  start_serial_bridge:=true \
  enable_test_publishers:=false \
  default_mode:=AUTO
```

Before `start_serial_bridge:=true`, stop any existing Waveshare `ugv_driver`, `app.py`, or other
serial bridge that may own the same port.

Useful status topics:

```bash
ros2 topic echo /waver/patrol_state
ros2 topic echo /waver/aerial_motion_state
ros2 topic echo /waver/aerial_target_active
ros2 topic echo /waver/target_tracking_state
ros2 topic echo /waver/auto_behavior_state
ros2 topic echo /waver/safety_state
ros2 topic echo /waver/battery_state_text
ros2 topic echo /waver/return_home_state
ros2 topic echo /cmd_vel
```

Check that `/cmd_vel` has exactly one publisher, `safety_cmd_mux_node`:

```bash
ros2 topic info -v /cmd_vel
```

Check LiDAR topics and adapter health:

```bash
ros2 topic list | grep -E "scan|cloud|PointCloud|detections"
ros2 topic hz /scan
ros2 topic hz /waver/lidar_objects
ros2 topic hz /lidar/detections
```

If the robot has a 3D LiDAR but no `/scan`, enable the scan adapter only after confirming the real
cloud topic:

```bash
ros2 launch waver_patrol waver_bird_autonomy.launch.py \
  enable_livox_scan_adapter:=true \
  pointcloud_topic:=/mid360_PointCloud2 \
  require_scan:=true \
  start_serial_bridge:=false \
  default_mode:=AUTO
```

For simple raw cloud object candidates:

```bash
ros2 launch waver_patrol waver_bird_autonomy.launch.py \
  enable_pointcloud_lidar_objects:=true \
  pointcloud_topic:=/mid360_PointCloud2 \
  require_scan:=true \
  start_serial_bridge:=false \
  default_mode:=AUTO
```

The deep-learning, sound-alert, and hardware-specific aerial object interfaces are stubs. They are
safe integration points, not finished perception or deterrent systems.

Real speaker output is disabled by default. Keep `enable_sound_output=false` until the amplifier,
speaker direction, legal/safety limits, and manual kill path have been separately validated.

Battery voltage thresholds in `config/waver_bird_autonomy.yaml` are placeholders. Recalibrate
`warning_voltage` and `critical_voltage` under actual battery load before relying on return-home.

Waypoint coordinates in `waypoints/waver_bird_patrol_demo.yaml` are odom-frame examples. Real
outdoor patrol needs coordinates verified against the actual map/localization origin.

## Final Real-Robot Safety Checklist

1. Confirm exactly one build-visible `waver_patrol` exists, preferably `src/FSD_Vehicle/src/waver_patrol`.
2. Build with `colcon build --packages-select waver_patrol --symlink-install`.
3. Confirm `/cmd_vel` publisher is only `safety_cmd_mux_node`.
4. Confirm `ugv_driver` and `serial_cmd_vel_bridge` are not both running.
5. Confirm `pcd_cluster_pkg` is not publishing `/cmd_vel`.
6. Confirm `/scan` is received and `require_scan=true`.
7. Confirm emergency stop and external stop force `/cmd_vel` to zero.
8. Confirm `/cmd_vel` timeout makes serial bridge send stop.
9. Lift wheels and verify left/right rotation direction.
10. Confirm target left gives `angular.z > 0`; target right gives `angular.z < 0`.
11. Confirm battery critical selects `BATTERY_RETURN`.
12. Confirm return-home arrival holds `HOLD_AT_HOME`.
13. Confirm test publishers are off on real launch.
14. Confirm `enable_sound_output=false`.

## Nav2 Radar Bird Mission Stack

The mission-experiment launch adds 4D radar target interruption, Nav2 mission
goals, safe sound-task stubs, and paper-oriented CSV logging:

```text
4D radar / external watcher
  -> radar_command_bridge_node
  -> /waver/radar_target_goal
  -> target_goal_manager_node
  -> /waver/object_mission_goal
  -> mission_patrol_manager_node
  -> Nav2 NavigateToPose action
  -> Nav2 controller output remapped to /waver/cmd_vel_nav2
  -> safety_cmd_mux_node
  -> /cmd_vel
  -> serial_cmd_vel_bridge
  -> WAVE ROVER JSON serial command
```

Final `/cmd_vel` must have exactly one publisher:

```bash
ros2 topic info -v /cmd_vel
```

Expected publisher: `safety_cmd_mux_node` only. Nav2, pcd_cluster_pkg, keyboard
teleop, Gazebo helpers, radar, LiDAR, camera, deep-learning, and test publishers
must not publish final `/cmd_vel` directly. Nav2 output is remapped to
`/waver/cmd_vel_nav2`.

### Mission Behavior

`mission_patrol_manager_node` repeats `waypoints/waver_nav2_patrol_mission.yaml`.
When radar/object mission goal appears, it stores the interrupted waypoint,
sends the object goal through Nav2, waits for classification, requests the safe
sound stub only if the target is a bird, returns to the interrupted waypoint,
and resumes patrol. Battery return preempts every mission state.

2D `LaserScan` has no z value. `/scan` is only for ground obstacle hard-stop and
slow-down. Height-based target decisions must use `PoseStamped`,
`PointStamped`, `PoseArray`, or `PointCloud2` derived interfaces.

### Hardware LiDAR Path

For real hardware, first discover the actual LiDAR topics and frame names:

```bash
ros2 topic list | grep -E "scan|cloud|PointCloud|mid360|livox|unilidar|detections"
ros2 topic hz /scan
ros2 topic hz /mid360_PointCloud2
ros2 run tf2_ros tf2_echo base_link livox_frame
```

If the robot already has a reliable 2D `/scan`, leave the PointCloud adapter off:

```bash
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=false \
  enable_test_publishers:=false \
  default_mode:=STANDBY
```

If the robot has only a 3D LiDAR PointCloud2 topic, enable both adapters after
confirming the real topic name. The scan adapter is for final safety stop only;
the object adapter is for elevated object candidates:

```bash
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  enable_livox_scan_adapter:=true \
  enable_pointcloud_lidar_objects:=true \
  pointcloud_topic:=/mid360_PointCloud2 \
  scan_topic:=/scan \
  start_serial_bridge:=false \
  enable_test_publishers:=false \
  default_mode:=STANDBY
```

If the LiDAR frame axes differ, adjust `scan_forward_axis`, `scan_lateral_axis`,
`scan_height_axis`, and `scan_positive_lateral_is_left`. For Waver's base frame,
the expected convention is `x` forward, `y` left, and `z` up.

### Moving Object Map Coordinates

LiDAR object candidates are now separated into raw sensor-frame data and shared
SLAM-frame data:

```text
raw LiDAR / 3D detector
  -> /waver/lidar_objects              # PoseArray in LiDAR/base frame
  -> moving_object_map_transform_node  # tf2 transform
  -> /waver/lidar_objects_map          # PoseArray in map or odom
  -> /waver/moving_objects_map_marker  # RViz MarkerArray
```

Run the transform alone:

```bash
ros2 launch waver_patrol moving_object_map_transform.launch.py \
  input_type:=pose_array \
  input_topic:=/waver/lidar_objects \
  target_frame:=map \
  fallback_frame:=odom
```

If `map` is not available yet, test in odom:

```bash
ros2 launch waver_patrol moving_object_map_transform.launch.py \
  input_type:=pose_array \
  input_topic:=/waver/lidar_objects \
  target_frame:=odom \
  fallback_frame:=
```

In the full mission launch:

```bash
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  enable_pointcloud_lidar_objects:=true \
  enable_moving_object_map_transform:=true \
  pointcloud_topic:=/mid360_PointCloud2 \
  require_scan:=true \
  start_serial_bridge:=false
```

The transform node does not publish `/cmd_vel`, `/goal_pose`, or
`/waver/object_mission_goal`. A validation-only relay can publish
`/waver/lidar_object_goal_preview` so you can confirm that the coordinate is
usable as a `PoseStamped` goal without letting LiDAR detections override 4D radar
mission commands.

See `docs/moving_object_map_transform.md` for the full topic/frame procedure.

No weapon, explosive, pyrotechnic, firing, or high-power sound hardware is
controlled here. `sound_alert_stub` defaults to `enable_sound_output=false` and
only publishes state plus `/waver/sound_task_done`. Any real gunshot-like or
deterrent sound requires a separate approved speaker driver after legal, site,
operator, wildlife, and noise-safety review.

### Visual Remote Integration

The `ugv_tools` visual remote is now a mode/manual-override panel for this stack.
Its AUTO button publishes `/waver/mode=AUTO`; it does not start another `/cmd_vel`
publisher in the default operator launch. During AUTO, arrow/WASD input publishes
only `/waver/manual_cmd_vel`, so `safety_cmd_mux_node` temporarily prioritizes the
operator command and then returns to `/waver/cmd_vel_nav2` when the key is released.

Operator PC or Jetson desktop:

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  require_scan:=true \
  auto_mode_strategy:=mission_nav2
```

The mission backend must already be running:

```bash
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=false \
  enable_test_publishers:=false \
  default_mode:=STANDBY \
  remap_nav2_cmd_vel:=true
```

Before enabling serial, verify final command ownership:

```bash
ros2 topic info -v /cmd_vel
```

Expected: exactly one publisher, `safety_cmd_mux_node`.

### Gazebo Mission Smoke Test

For headless Gazebo validation before the real robot, use the dedicated mission
smoke launch. It uses `waver_flat.world`, the minimal Waver model, fake radar and
fake camera publishers, and a Gazebo-only `simple_nav2_cmd_sim_node` when
`use_nav2:=false`. That simulator is only a Nav2 stand-in for mission wiring; real
deployment must use `use_nav2:=true`.

```bash
ros2 launch waver_patrol waver_gazebo_nav2_radar_bird_mission.launch.py \
  use_minimal_model:=true \
  use_gui:=false \
  use_nav2:=false \
  enable_simple_nav2_cmd_sim:=true \
  require_scan:=false \
  enable_test_publishers:=true \
  enable_experiment_logger:=true
```

`require_scan:=false` is allowed only for this smoke test. Real driving should use
`require_scan:=true` and the actual LiDAR `/scan` or 3D-LiDAR-to-scan adapter.

### Mission Build And Run

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install -i --from-paths src/FSD_Vehicle/src/waver_patrol --rosdistro humble -y
colcon build --packages-select waver_patrol --symlink-install
source install/setup.bash
```

Desk state-machine test, no robot movement, no serial, no real Nav2:

```bash
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=false \
  enable_test_publishers:=true \
  require_scan:=false \
  start_serial_bridge:=false \
  default_mode:=AUTO \
  enable_experiment_logger:=true
```

`require_scan:=false` is only for desk tests.

Nav2 indoor test:

```bash
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=false \
  enable_test_publishers:=false \
  default_mode:=AUTO \
  enable_experiment_logger:=true
```

Gazebo headless smoke test using the existing `ugv_gazebo` world and a
render-lightweight Waver safety model:

```bash
ros2 launch waver_patrol waver_gazebo_nav2_radar_bird_mission.launch.py \
  use_nav2:=false \
  require_scan:=false \
  enable_test_publishers:=true \
  spawn_robot:=true \
  use_minimal_model:=true \
  spawn_birds:=true \
  use_gui:=false
```

Use `use_minimal_model:=false` only on a machine where the original `ugv_rover`
Gazebo render/sensor plugins are known to work. In this headless test
environment, the original model can crash `gzserver` during render scene
initialization; the minimal safety model is therefore the recommended smoke
test target.

Real robot serial test, only after stopping `ugv_driver`, `app.py`, and other
serial bridges:

```bash
ros2 topic info -v /cmd_vel
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=true \
  enable_test_publishers:=false \
  default_mode:=AUTO
```

Radar and classification topic tests:

```bash
ros2 topic pub /radar4d/target_pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.5}, orientation: {w: 1.0}}}"

ros2 topic pub /waver/external_target_class std_msgs/msg/String "{data: 'bird'}"
ros2 topic pub /waver/external_target_confidence std_msgs/msg/Float32 "{data: 0.92}"
```

### Experiment CSV And Rosbag

CSV output:

```bash
ls -R ~/ros2_ws/waver_experiments
head -n 5 ~/ros2_ws/waver_experiments/*/mission_events.csv
head -n 5 ~/ros2_ws/waver_experiments/*/radar_targets.csv
head -n 5 ~/ros2_ws/waver_experiments/*/experiment_summary.csv
```

Raw rosbag recording:

```bash
~/ros2_ws/src/FSD_Vehicle/src/waver_patrol/scripts/record_waver_experiment_bag.sh
```

### Research References Used

Official references:

- ROS 2 Humble rosbag2 record/replay documentation.
- Nav2 Humble NavigateToPose action and navigation action APIs.
- Nav2 Collision Monitor and Velocity Smoother documentation.
- ROS 2 tf2 geometry transform documentation.

Papers/datasets/open source:

- KAIST AVELab K-Radar: full 4D radar tensor dataset with camera/LiDAR/radar fusion notes.
- TJ4DRadSet: 4D radar dataset with synchronized LiDAR/camera/radar frames.
- Awesome Radar-Camera Fusion: survey/repository index for radar-camera fusion.
- WaterScenes and recent 4D radar-camera fusion papers for dataset/logging inspiration.
- rosbag2 GitHub for raw replay/data collection practices.

Acoustic bird deterrence note: acoustic deterrents can habituate and may be
regulated by site/noise/wildlife rules, so Waver logs a safe sound task stub
instead of driving real output hardware.
15. Test only in a controlled empty area with a physical E-Stop path.

## Gazebo Bird Autonomy Test

This workspace also contains `ugv_gazebo` under `src/FSD_Vehicle/src/ugv_main/ugv_gazebo`.
Use Waver's Gazebo launch to keep the same command path as the real robot: Waver candidate
commands go through `auto_behavior_mux_node`, then `safety_cmd_mux_node`, and only then `/cmd_vel`.

```bash
cd /home/chotaehyun/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --paths src/FSD_Vehicle/src/waver_patrol src/FSD_Vehicle/src/ugv_main/ugv_gazebo \
  --packages-select waver_patrol ugv_gazebo --symlink-install
source install/setup.bash

ros2 launch waver_patrol waver_gazebo_bird_autonomy.launch.py \
  use_gui:=false \
  require_scan:=true \
  enable_scripted_keyboard_test:=false
```

Gazebo notes:

- `waver_empty.world` includes `libgazebo_ros_state.so` so the existing Gazebo `bird_manager.py`
  can move the bird models. The original file was backed up as
  `waver_empty.world.bak_waver_state_20260518`.
- In this local Gazebo install, the UGV SDF references `libros2_livox.so`, but that plugin is not
  installed. The Gazebo launch therefore uses `/camera/points` and
  `livox_pointcloud_to_scan_node` to create the safety `/scan`.
- If your Jetson or PC has the Livox Gazebo plugin working, run with:

```bash
ros2 launch waver_patrol waver_gazebo_bird_autonomy.launch.py \
  pointcloud_topic:=/mid360 \
  scan_forward_axis:=x \
  scan_lateral_axis:=y \
  scan_height_axis:=z \
  scan_positive_lateral_is_left:=true
```

Check the same topics before touching the real robot:

```bash
ros2 topic info -v /cmd_vel
ros2 topic echo /waver/safety_state
ros2 topic echo /waver/auto_behavior_state
ros2 topic echo /waver/aerial_target_active
ros2 topic echo /waver/target_tracking_state
ros2 topic echo /waver/gazebo_bird_bridge_state
ros2 topic echo /waver/livox_scan_adapter_state
```

`ros2 topic info -v /cmd_vel` must show exactly one publisher: `safety_cmd_mux_node`.

## Tests

```bash
cd /home/chotaehyun/ros2_ws/src/FSD_Vehicle/src/waver_patrol
pytest -q
cd /home/chotaehyun/ros2_ws
colcon test --packages-select waver_patrol
```

Core tests use fake serial and do not require ROS2 runtime.

## Rollback Policy

This package does not modify upstream Waveshare files. If a future change must patch an upstream
file, first create `file.bak`, record the reason, show a unified diff, and add rollback commands here:

```bash
cp file.bak file
colcon build --packages-select <package>
```
