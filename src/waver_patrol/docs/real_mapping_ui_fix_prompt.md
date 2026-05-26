# Waver Real Mapping UI Fix Prompt

This prompt is the working contract for fixing the current Waver pre-real workspace.
It must be executed against the existing `FSD_Vehicle` `jo` branch without creating a new project.

## Goal

Make the existing ROS 2 Humble Waver stack suitable for pre-real validation:

1. The operator UI keeps WASD and arrow-key manual drive.
2. Manual drive publishes only `/waver/manual_cmd_vel` in the real profile.
3. Final `/cmd_vel` is published only by `safety_cmd_mux_node`.
4. Gazebo airport world uses `ugv_gazebo/worlds/ugv_world.world`.
5. Gazebo robot model uses `ugv_gazebo/models/ugv_rover/model.sdf`.
6. Pressing `SLAM MAPPING` in the UI clears the old displayed map and enters `MAPPING_AUTO`.
7. During mapping, the UI shows live `/map` as `SLAM_LIVE`.
8. During mapping, Waver can move by WASD manual override through the safety mux.
9. During mapping, optional low-speed autonomous mapping patrol is allowed only through `/waver/cmd_vel_nav2`.
10. Pressing `SAVE MAP` saves the current live map but does not automatically apply it.
11. Pressing `APPLY FIXED MAP` or `LOAD MAP` applies the saved map back to `/map`, switches the UI to `MAP_FIXED`, and returns mode to `STANDBY`.
12. Pressing `START PATROL` after map apply starts fixed-map patrol from the mission backend.
13. Elevated dynamic object mission remains height based: `object_height_m >= 3.0`, `z_valid=true`, dynamic in map/odom frame, and ego-motion compensated.
14. Object target missions are disabled during mapping by default.

## External Design Basis

- ROS 2 Humble `tf2` documentation: transform object coordinates into a common world frame before comparing motion.
- ROS 2 Humble rosbag2 documentation: record selected ROS topics for replayable experiment evidence.
- Nav2 Waypoint Follower documentation: waypoint patrol should use Nav2 actions or existing mission manager rather than direct `/cmd_vel`.
- Nav2 Collision Monitor and Velocity Smoother documentation: final velocity should be bounded and safety-gated before hardware.
- SLAM Toolbox and standard SLAM practice: live mapping requires `/scan`, `/odom`, `/tf`, and a stable map/odom/base_link tree.
- 3D LiDAR dynamic object detection literature: apparent sensor-frame motion must be separated from ego-motion; dynamic/static classification should be done after frame compensation.

## Must-Fix Issues

### 1. Unsafe Default AUTO

Real mission launch and mission manager must default to `STANDBY`.
Gazebo validation may explicitly set `AUTO` only for scripted validation launches.

Required changes:

- `waver_nav2_radar_bird_mission.launch.py`: `default_mode:=STANDBY`.
- `mission_patrol_manager_node.py`: parameter default `default_mode="STANDBY"`.
- `safety_cmd_mux_node.py`: parameter default `mode_default="STANDBY"`.

### 2. UI SLAM Mapping Workflow

`SLAM MAPPING` button must:

- publish `START_MAPPING`;
- clear displayed old map, paths, traces, goals, lidar objects, elevated targets;
- set UI map mode `SLAM_LIVE`;
- request `MAPPING_AUTO`;
- start only the configured mapping backend, never Gazebo, from the operator panel;
- preserve WASD manual override through `/waver/manual_cmd_vel`.

`SAVE MAP` must:

- publish `SAVE_MAP`;
- require a live map;
- save to `~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml` plus archive copy;
- publish `/waver/map_saved_path`;
- set `/waver/map_apply_state=MAP_SAVED_NOT_APPLIED`;
- not apply the map automatically.

`APPLY FIXED MAP` must:

- publish `APPLY_FIXED_MAP`;
- load the saved YAML/PGM;
- republish the saved fixed map on `/map` with transient-local QoS for late UI subscribers;
- set `/waver/map_apply_state=MAP_FIXED_READY`;
- set `/waver/mapping_active=false`;
- set `/waver/mode=STANDBY`;
- update UI to `MAP_FIXED`.

### 3. Gazebo Mapping Test

The Gazebo mapping launch must:

- use `ugv_world.world`;
- spawn `ugv_rover`;
- default to `STANDBY`;
- not move until the operator UI sends commands;
- support an automated UI smoke script for regression testing;
- use explicit SAVE/APPLY workflow, with `auto_save_on_complete=false` and `auto_apply_on_save=false`.

### 4. Safety and Command Path

Allowed command path:

```text
UI WASD -> /waver/manual_cmd_vel -> safety_cmd_mux_node -> /cmd_vel -> one serial bridge
Nav2/mapping/patrol -> /waver/cmd_vel_nav2 -> safety_cmd_mux_node -> /cmd_vel -> one serial bridge
```

Forbidden path:

```text
UI -> /cmd_vel
Nav2 -> legacy ugv_driver -> serial
perception/cluster -> /cmd_vel
```

### 5. Verification

Run:

```bash
python3 - <<'PY'
import py_compile, pathlib
errs=[]
for p in pathlib.Path('src/FSD_Vehicle/src').rglob('*.py'):
    if '__pycache__' in p.parts or '.pytest_cache' in p.parts:
        continue
    try:
        py_compile.compile(str(p), doraise=True)
    except Exception as e:
        errs.append((str(p), repr(e)))
print('errors:', errs)
assert not errs
PY

colcon build --packages-select ugv_tools waver_patrol --symlink-install
```

Then run Gazebo airport mapping smoke:

```bash
ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=false \
  use_operator_panel:=true \
  demo_script:=mapping_workflow_smoke \
  demo_close_on_finish:=true \
  reveal_duration_sec:=5.0
```

Success criteria:

- `ugv_rover` is spawned in `ugv_world.world`.
- UI starts in STANDBY and does not directly publish `/cmd_vel`.
- `SLAM MAPPING` transitions to `MAPPING_AUTO`.
- WASD/manual demo publishes `/waver/manual_cmd_vel`.
- safety mux is the final `/cmd_vel` publisher.
- live `/map` appears.
- `SAVE_MAP` creates `~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml`.
- `APPLY_FIXED_MAP` republishes fixed `/map` and sets `MAP_FIXED_READY`.
- The UI remains alive and shows map/pose/path/status.

## Release Gate

The project is not wheel-on ready until:

- py_compile passes;
- focused build passes;
- Gazebo airport mapping smoke passes;
- H1/H2/H3 height target validation passes;
- `/cmd_vel` publisher count is one;
- no `ugv_driver` is launched in the Waver real profile;
- the operator UI map workflow works without Gazebo-specific defaults in the real profile.
