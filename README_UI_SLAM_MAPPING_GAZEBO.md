# Waver UI SLAM Mapping Gazebo

Workspace: `~/ros2_ws5/FSD_Vehicle`

This profile validates the operator-panel SLAM mapping workflow without touching
the `~/ros2_ws5` field-success setup.

## Build

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select waver_patrol ugv_tools ugv_gazebo ugv_description
source install/setup.bash
```

## Manual Launch

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch ugv_gazebo ugv_gazebo_ui_slam_mapping.launch.py \
  use_sim_time:=true \
  use_gui:=true \
  mapping_backend:=slam_toolbox \
  start_remote_panel:=true \
  save_dir:=~/ros2_ws5/FSD_Vehicle/maps
```

UI buttons:

- `SLAM MAPPING`: clears old UI map state and starts the live SLAM map workflow.
- WASD/buttons: publish `/waver/manual_cmd_vel` only.
- `SAVE MAP`: saves the live `/map` to `~/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml`.
- `APPLY FIXED MAP`: publishes the saved map on `/map_fixed`.
- `START PATROL`: starts the 4 m straight, then 7 m square odom waypoint route.

## Automated Smoke

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/run_ui_slam_mapping_gazebo_smoke.sh
```

Longer mapping plus patrol:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/run_ui_slam_mapping_then_patrol_gazebo_test.sh
```

## Authority Checks

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /waver/mode
ros2 topic info -v /scan_slam
ros2 topic info -v /scan_safety
ros2 topic info -v /map
ros2 topic echo --once /waver/mapping_backend_state
python3 scripts/check_remote_ui_slam_mapping_result.py --strict
```

Expected authorities:

- `/cmd_vel`: one publisher, `safety_cmd_mux_node`.
- `/waver/mode`: one publisher, `mission_patrol_manager_node`.
- `/map`: one publisher during mapping, `slam_toolbox`.
- `/scan_slam`: one publisher, `scan_to_slam_alias`.
- `/scan_safety`: one publisher, `scan_to_safety_alias`.

## Real Prep

This launch is Gazebo-only. Real prep topic contracts are documented in:

- `src/ugv_main/ugv_gazebo/param/ui_slam/real_prep.yaml`
- `src/ugv_main/ugv_gazebo/param/ui_slam/topic_remaps_real_prep.yaml`

Do not run Gazebo-only publishers in real profile.
