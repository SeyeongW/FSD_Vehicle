# Final Handover Summary

Active workspace: `~/ros2_ws5/FSD_Vehicle`
Active repository: `~/ros2_ws5/FSD_Vehicle`
Branch: `jo`
Reference commit during validation: `a6bf8af`

This handover is limited to the `jo` branch. Do not use
`~/ros2_ws5/FSD_Vehicle` for the current release workflow.

## What This Version Provides

- Existing `ugv_gazebo` airport world and `ugv_rover` SDF remain the Gazebo simulation fixtures.
- Existing `ugv_slam` mapping launch files remain the LiDAR-only SLAM entry point.
- Existing `ugv_nav`/Nav2 launch files remain the saved-map localization/navigation entry point.
- `ugv_tools` operator panel keeps WASD/manual driving, but manual commands go to `/waver/manual_cmd_vel`; the UI does not directly publish final `/cmd_vel` when `publish_direct_cmd_vel:=false`.
- `waver_patrol` owns mapping workflow, internal map save/apply fallback, safety mux, Gazebo validation, and paper-ready trial export.
- Final `/cmd_vel` is owned by `safety_cmd_mux_node` in the validated Gazebo/UI workflow.

## Mapping Workflow

```text
Gazebo ugv_world + ugv_rover
  -> /scan + /odom + /tf
  -> UI SLAM MAPPING command
  -> LiDAR-only SLAM backend
  -> live /map shown as SLAM_LIVE
  -> SAVE MAP
  -> ~/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml/.pgm
  -> APPLY FIXED MAP
  -> fixed /map shown in operator UI
  -> START PATROL / STOP / E-STOP checks
```

Depth-camera SLAM is not used in the validated mapping workflow.

## Target Definition

The target is **not** an object that moved 3 m. The target is:

```text
object_height_m >= target_min_height_m
z_valid == true
dynamic_filter_pass == true
ego_motion_compensated == true
```

Default `target_min_height_m` is `3.0`. A 2D LaserScan-only object must not be
accepted as an elevated target because it has no real z-source provenance.

## Perception Path

```text
3D PointCloud2 / Gazebo 3D PoseArray / custom 3D source
  -> /waver/lidar_objects
  -> moving_object_map_transform_node
  -> /waver/lidar_objects_map
  -> moving_object_motion_filter_node
  -> /waver/elevated_dynamic_targets
  -> target_goal_manager_node
  -> /waver/object_mission_goal
  -> mission_patrol_manager_node
```

`moving_object_motion_filter_node` now requires finite z plus accepted 3D source
provenance by default. LaserScan-only or unknown z sources are classified as
height unknown and do not trigger a mission.

## Gazebo/UI Evidence Policy

This handover no longer claims repeated-trial performance results. Historical
Gazebo/UI outputs may remain in local development folders, but they are not paper
results unless revalidated with the current schema and packaged as a separate
evidence package.

Allowed current wording:

- Source/static contracts and unit tests passed when recorded in `reports/local_validation_summary.md`.
- Optional single-run Gazebo smoke logs may support simulation-smoke claims only when `final_status=PASS`.
- Raw experiment data is required before reporting performance tables, detector metrics, mission success rates, or map-quality statistics.

The repeated `libros2_livox.so` plugin warning must not be hidden. If the Livox
Gazebo plugin is unavailable, only scan-mapper fallback smoke behavior may be
described. Real 3D target work still needs the actual Livox/Mid360 driver or
another verified 3D source.

## Build Status

Verified commands:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
python3 -m compileall -q src/waver_patrol src/ugv_main/ugv_tools
colcon build --packages-select \
  ugv_description ugv_bringup ugv_tools waver_patrol ugv_slam ugv_gazebo \
  --symlink-install
```

Result: compileall passed and six focused packages built successfully.

## Run Commands

Gazebo only:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=true \
  use_operator_panel:=false \
  robot_spawn_x:=0.0 \
  robot_spawn_y:=0.0 \
  robot_spawn_z:=0.15
```

Operator UI only:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_topic:=/map \
  map_display_mode:=auto \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  require_scan:=false \
  auto_mode_strategy:=mission_nav2 \
  publish_direct_cmd_vel:=false
```

Optional local Gazebo/UI smoke runner. Do not cite this as paper performance
evidence unless its raw outputs are packaged separately and audited:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

TRIAL_TIMEOUT_SEC=220 \
OUTPUT_ROOT=$HOME/ros2_ws5/FSD_Vehicle/experiments_result/local_ai_gazebo_ui_smoke \
bash src/waver_patrol/scripts/run_ai_gazebo_ui_mapping_trials.sh
```

View generated local summary if present:

```bash
find experiments_result/local_ai_gazebo_ui_smoke -maxdepth 2 -type f | sort
```

Safety check while backend is running:

```bash
ros2 topic info -v /cmd_vel
ros2 topic echo /waver/safety_state
ros2 topic echo /waver/mission_state
```

Expected `/cmd_vel` publisher: `safety_cmd_mux_node` exactly once.

## Applicability Judgment

The current evidence supports **simulation-based conditional pass** for supervised
pre-deployment work. It does **not** prove unrestricted real-vehicle operation.

Real wheel-on PASS still requires:

- real_vehicle_precheck PASS
- rosbag replay PASS
- wheel-off HIL PASS
- hardware E-STOP PASS
- software E-STOP PASS
- scan/odom/TF stale-stop PASS
- supervised closed-area low-speed test PASS

## Remaining Manual Checks

- Confirm actual 3D LiDAR topic, frame, and z-source provenance on Jetson.
- Confirm `map -> odom -> base_link -> lidar_frame` TF on the real robot.
- Confirm `/cmd_vel` publisher is one and named `safety_cmd_mux_node`.
- Confirm serial bridge is disabled until real precheck and wheel-off HIL are complete.
- Confirm operator panel shows `MAP_FIXED` after saved-map apply and that the map does not rotate with Waver yaw.
- Confirm sound output remains disabled until legal/safety approval.
