# Gazebo Mapping Mode And Dynamic Obstacle Avoidance

This document records the pre-real Gazebo smoke checks added for the Waver
operator UI and navigation mechanism.

## Mapping Mode

Launch:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=122

ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=true \
  use_operator_panel:=true \
  reveal_duration_sec:=20.0
```

The launch uses `ugv_gazebo/worlds/ugv_world.world` and
`ugv_gazebo/models/ugv_rover/model.sdf`. It starts a Gazebo-only mapping
preview node that publishes a live-changing `/map`, saves it on `SAVE_MAP`,
and applies the saved map back to `/map` for the operator panel.

Validation:

```bash
python3 ~/ros2_ws/src/FSD_Vehicle/src/waver_patrol/scripts/run_mapping_mode_check.py \
  --output-root ~/ros2_ws/experiments_result \
  --experiment-name mapping_mode_manual \
  --duration-sec 14 \
  --save-after-sec 9
```

Expected outputs:

- `/map` known ratio increases while mapping is active.
- `/waver/map_apply_state` reports `MAP_SAVED` and `MAP_APPLIED`.
- `~/ros2_ws/maps/waver_latest_map.yaml` is created.
- The operator UI shows `SLAM_LIVE` while mapping and `MAP_FIXED` after apply.

Important: this mapping preview is simulation-only. Real mapping still uses
`ugv_slam` through Cartographer or Gmapping.

## Dynamic Obstacle Avoidance Smoke Test

Launch:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=124

ros2 launch waver_patrol gazebo_dynamic_obstacle_avoidance.launch.py \
  use_gui:=true \
  require_scan:=false
```

This launch uses `ugv_world.world`, spawns `ugv_rover`, spawns
`dynamic_test_box`, publishes the obstacle on `/waver/dynamic_obstacle_map`,
and enables Gazebo-only detour preview in `/plan` plus simple command
simulation through the safety mux.

Validation:

```bash
python3 ~/ros2_ws/src/FSD_Vehicle/src/waver_patrol/scripts/run_dynamic_obstacle_avoidance_check.py \
  --output-root ~/ros2_ws/experiments_result \
  --experiment-name dynamic_obstacle_avoidance_manual \
  --duration-sec 22
```

Expected outputs:

- `/model_states` contains `ugv_rover` and `dynamic_test_box`.
- `/waver/gazebo_dynamic_obstacle_state` reports the obstacle motion.
- `/waver/sim_nav2_state` reaches an `AVOIDING` state at least once.
- `/plan` contains a detour waypoint when the obstacle enters the corridor.
- `/cmd_vel` has exactly one publisher: `safety_cmd_mux_node`.
- `experiment_summary.csv` reports `overall_success=True`.

This is a Gazebo smoke test for the mechanism and operator visualization. It
does not replace final real Nav2 replanning validation with the full
`ugv_nav` stack and real sensors.

## Operator Panel

Run against either Gazebo mode:

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_topic:=/map \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  require_scan:=false \
  map_display_mode:=map_fixed \
  publish_direct_cmd_vel:=false
```

The panel subscribes to `/map`, `/odom`, `/amcl_pose`, `/plan`,
`/local_plan`, `/waver/lidar_objects_map`, `/waver/elevated_dynamic_targets`,
mission state, safety state, camera state, and sound state. It publishes
operator commands to `/waver/mission_command` and manual candidate velocity to
`/waver/manual_cmd_vel`; it does not publish `/cmd_vel` unless
`publish_direct_cmd_vel:=true` is explicitly set for legacy tests.
