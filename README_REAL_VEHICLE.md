# Waver Real-Vehicle Bird Autonomy Checklist

This repository must be used on branch `jo` only. The real profile is designed for a low-speed, supervised, closed-area Waver UGV test. It does not enable wheel-on driving by default.

## Command Chain

Default mode is `safety_mux_final`:

```text
Nav2 controller
  -> /waver/cmd_vel_nav2_raw
  -> nav2_velocity_smoother
  -> /waver/cmd_vel_nav2_smooth
  -> safety_cmd_mux_node
  -> /cmd_vel
  -> waver_base_driver_node or serial_cmd_vel_bridge
```

The final `/cmd_vel` publisher must be exactly one node: `safety_cmd_mux_node`. The operator UI publishes WASD/manual commands only to `/waver/manual_cmd_vel` and mode requests to `/waver/mode_cmd`; `mission_patrol_manager_node` owns `/waver/mode`.

For real wheel-off/wheel-on tests, prefer `enable_waver_base_driver:=true`. It owns one serial port, subscribes only to final `/cmd_vel`, and publishes `/odom_raw`, `/imu/data_raw`, `/voltage`, `/waver/base_driver_state`, and `/waver/serial_owner_state`.

## Build

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
rosdep install --from-paths FSD_Vehicle/src --ignore-src -r -y
bash FSD_Vehicle/src/waver_patrol/scripts/waver_duplicate_package_check.sh
colcon build --symlink-install --packages-select \
  waver_patrol ugv_bringup ugv_base_node ugv_tools ugv_nav
source install/setup.bash
```

## Dry Run, No Serial

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  start_serial_bridge:=false \
  enable_waver_base_driver:=false \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  scan_source:=mid360 \
  odom_source:=ekf \
  map:=$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml \
  bird_model_path:=$HOME/models/bird_yolov8n.pt
```

If `bird_model_path` is empty or missing, the detector publishes `MODEL_MISSING` and `bird_confirmed=false`; patrol can be checked, but bird approach is blocked.

## Operator UI

```bash
ros2 launch ugv_tools waver_operator_panel.launch.py \
  profile:=real \
  publish_direct_cmd_vel:=false \
  map_topic:=/map \
  map_display_mode:=auto \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  camera_detection_status_topic:=/waver/bird_detector_state
```

## Preflight Checks

```bash
bash FSD_Vehicle/src/waver_patrol/scripts/waver_real_preflight_check.sh --strict
bash FSD_Vehicle/src/waver_patrol/scripts/waver_cmd_chain_check.sh
bash FSD_Vehicle/src/waver_patrol/scripts/waver_bird_autonomy_health_check.sh
```

Do not enable serial if any check fails.

## Wheel-Off Test

Only run this with wheels off the ground and a physical E-stop available.

```bash
ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  enable_waver_base_driver:=true \
  start_serial_bridge:=false \
  serial_port:=/dev/serial/by-id/<WAVER_SERIAL_ID> \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  scan_source:=mid360 \
  odom_source:=ekf \
  map:=$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml \
  bird_model_path:=$HOME/models/bird_yolov8n.pt
```

Verify manual direction, STOP, E-stop, serial reconnect stop burst, scan stale stop, and `/cmd_vel` single publisher before any wheel-on test.

## Closed-Area Low-Speed Wheel-On

Only after dry-run, preflight, and wheel-off tests pass:

```bash
ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  enable_waver_base_driver:=true \
  start_serial_bridge:=false \
  serial_port:=/dev/serial/by-id/<WAVER_SERIAL_ID> \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  scan_source:=mid360 \
  odom_source:=ekf \
  map:=$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml \
  bird_model_path:=$HOME/models/bird_yolov8n.pt
```

Start in `STANDBY`. The operator must explicitly press START PATROL in the UI. Bird approach requires `bird_confirmed=true`, valid 3D fusion, elevated height, dynamic motion, fresh robot pose, and a passing safety state.

## Known Remaining Risks

- Camera-LiDAR extrinsic calibration must be measured on the real vehicle.
- Bird model quality depends on an airport/runway dataset and false-positive testing.
- Mid360 frame axes must be verified with TF and pointcloud visualization.
- Battery thresholds must be calibrated under load.
- Hardware E-stop and serial protocol direction must be validated wheel-off first.
