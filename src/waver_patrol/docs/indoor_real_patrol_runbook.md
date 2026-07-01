# Indoor Real Patrol Runbook

This runbook is for a supervised low-speed indoor Waver UGV check before full bird-autonomy operation.

## Purpose

Validate the minimum indoor real-robot patrol stack: map/localization, scan safety, odom, final command mux, mission mode authority, and low-speed waypoint patrol.

## Do Not

- Do not run wheel-on before strict preflight and wheel-off direction checks pass.
- Do not run fake/test/Gazebo publishers in this profile.
- Do not publish motor commands from automation scripts during software-only validation.
- Do not raise first-test speed limits above `0.05 m/s` linear or `0.20 rad/s` angular.

## Safety Preconditions

- Branch is `jo`.
- Physical E-stop is reachable.
- Wheels are off the ground for the first serial direction test.
- No fake/test/Gazebo publishers are running.
- Final `/cmd_vel` publisher is exactly `safety_cmd_mux_node`.
- `/waver/mode` publisher is exactly `mission_patrol_manager_node`.
- First wheel-on speed limits remain:
  - linear <= `0.05 m/s`
  - angular <= `0.20 rad/s`

## Build

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select waver_patrol ugv_tools ugv_nav ugv_base_node ugv_bringup
source install/setup.bash
```

## Pre-ROS Unit Checks

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
bash scripts/run_no_ros_unit_tests.sh
python3 scripts/waver_contract_check.py

cd src/waver_patrol
PYTHONPATH=. pytest -q test
```

## Clean Graph

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/waver_patrol/scripts/waver_cleanup_stale_nodes.sh
bash src/waver_patrol/scripts/waver_clean_graph_check.sh
```

## Dry Run Without Serial

```bash
ros2 launch waver_patrol waver_indoor_patrol_real.launch.py \
  use_nav2:=true \
  use_rviz:=false \
  enable_waver_base_driver:=false \
  default_mode:=STANDBY \
  map:=$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml
```

## Wheel-Off Serial Check

```bash
ros2 launch waver_patrol waver_indoor_patrol_real.launch.py \
  enable_waver_base_driver:=true \
  serial_port:=/dev/serial/by-id/<WAVER_SERIAL_ID> \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  map:=$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml
```

In another terminal:

```bash
bash src/waver_patrol/scripts/waver_cmd_chain_check.sh
WAVER_PREFLIGHT_PROFILE=indoor_patrol \
WAVER_PREFLIGHT_REQUIRE_CAMERA=0 \
WAVER_PREFLIGHT_REQUIRE_BIRD=0 \
WAVER_PREFLIGHT_REQUIRE_BATTERY=0 \
bash src/waver_patrol/scripts/waver_real_preflight_check.sh --strict
```

## Scan Quality

```bash
ros2 topic hz /scan
ros2 topic echo --once /waver/livox_scan_adapter_state
bash src/waver_patrol/scripts/waver_scan_quality_check.sh /scan --strict
```

The script reports Hz, finite count, finite ratio, front/rear minimum distance, and Livox adapter state.

## TF And Odom Sanity

Manual checks before wheel-on:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo --once /odom
```

Verify:

- `map -> odom -> base_link` exists.
- AMCL initial pose is set on the fixed map.
- Waypoints are inside the known safe map area.
- Wheel-off positive `linear.x` moves the robot forward.
- Wheel-off positive `angular.z` rotates in the expected direction.
- Odom direction matches physical motion.

## Status Helper

```bash
bash src/waver_patrol/scripts/waver_indoor_patrol_status.sh --strict
```

This helper only reads topics. It does not publish motor commands.

## Rosbag Record

```bash
ros2 bag record \
  /tf /tf_static \
  /odom /odom_raw \
  /cmd_vel \
  /scan \
  /waver/mode \
  /waver/safety_state \
  /waver/mission_state \
  /waver/livox_scan_adapter_state \
  /waver/base_driver_state \
  /waver/serial_owner_state \
  /waver/battery_safety_state
```

## Start Patrol

After preflight passes:

```bash
ros2 topic pub --once /waver/mode_cmd std_msgs/msg/String "{data: 'AUTO'}"
ros2 topic pub --once /waver/mission_command std_msgs/msg/String "{data: 'START_PATROL'}"
```

Stop:

```bash
ros2 topic pub --once /waver/mission_command std_msgs/msg/String "{data: 'STOP'}"
ros2 topic pub --once /waver/mode_cmd std_msgs/msg/String "{data: 'STANDBY'}"
```

## Emergency Stop And Clear

Emergency stop:

```bash
ros2 topic pub --once /waver/emergency_stop std_msgs/msg/Bool "{data: true}"
```

Clear emergency after the physical hazard is gone:

```bash
ros2 topic pub --once /waver/emergency_stop std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /waver/mission_command std_msgs/msg/String "{data: 'CLEAR_EMERGENCY_STOP'}"
```

## Experiment End Check

```bash
ros2 topic echo --once /cmd_vel
ros2 topic echo --once /waver/safety_state
bash src/waver_patrol/scripts/waver_clean_graph_check.sh
```

The final command should be zero after STOP/STANDBY.

## Field PC to Jetson UI Workflow

The existing SSH -> Jetson Docker -> Waver USB serial workflow is intentionally kept:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_docker_backend_start.sh

# second local PC terminal
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_local_ui_start.sh
```

## Final Wheel-On Checklist

- `/cmd_vel` publisher is exactly `safety_cmd_mux_node`.
- `/waver/mode` publisher is exactly `mission_patrol_manager_node`.
- `/scan` rate is at least 5 Hz.
- `/scan` front/rear sectors are clear.
- Livox adapter state is not degraded/stale/failed/no-points.
- Serial owner count is one or less.
- Emergency stop topic and physical E-stop have been tested wheel-off.
- First speed limits remain `0.05 m/s` and `0.20 rad/s`.
