# Waver Real-Vehicle Bringup

This repository is prepared for ROS 2 Humble Waver UGV field bringup on the
`jo` branch. It is not a one-command autonomous deployment. Real wheels,
serial motor output, sound output, and GPIO-like actuators must stay disabled
until the required readiness level passes.

## 1. Safety Warning

- Keep the robot lifted or wheels disconnected for driver checks.
- Keep a physical E-stop in the operator's hand for any wheel-on test.
- Do not enable sound hardware unless the hardware and local safety/legal
  acknowledgements are explicit.
- Do not run Gazebo, fake, mock, or test publishers in a real profile.
- Final `/cmd_vel` authority is `safety_cmd_mux_node` only.

## 2. Hardware Assumptions

Expected hardware path:

```text
Local PC UI
  -> SSH
  -> Jetson host
  -> Docker container fsd_dev_jetson
  -> ROS 2 nodes
  -> Waver USB serial or future UART serial
```

Expected real sensor topics:

- Mid360 pointcloud: `/livox/lidar`
- Mid360 IMU: `/livox/imu`
- Safety scan: `/scan_safety` or `/scan`
- EKF odom: `/odom`
- Base feedback: `/odom_raw`, `/imu/data_raw`, `/voltage`,
  `/waver/base_driver_state`, `/waver/serial_owner_state`

Use `/dev/serial/by-id/...` for motor serial. Avoid `/dev/ttyUSB0` in final
wheel-on work unless a single connected serial device has been verified.

## 3. Readiness Levels

The canonical level definitions are in
`docs/hardware_readiness_levels.md`.

- `L0_SOURCE_CHECK`: source/static/package checks, no hardware access.
- `L1_ROS_GRAPH_DRY_RUN`: real-profile graph, serial and sound disabled.
- `L2_SENSOR_ONLY_LIVE`: live LiDAR/TF with motors disabled.
- `L3_WHEEL_OFF_DRIVER`: base driver feedback with wheels off ground.
- `L4_WHEEL_ON_LOW_SPEED`: closed-area supervised low-speed wheel-on.
- `L5_AUTONOMOUS_PATROL`: guarded autonomous patrol after L4 evidence.

## 4. L0 Source Check

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash

python3 -m compileall -q scripts src/waver_patrol/waver_patrol
bash scripts/run_no_ros_unit_tests.sh
python3 scripts/waver_contract_check.py --require-git-branch jo
python3 scripts/waver_field_readiness_check.py --level L0 --strict --no-hardware
```

Expected final line:

```text
FIELD_READINESS=PASS
```

## 5. L1 Dry Run

L1 must not open the motor serial port.

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  start_serial_bridge:=false \
  enable_waver_base_driver:=false \
  default_mode:=STANDBY \
  require_scan:=false \
  enable_sound_deterrent:=false \
  enable_sound_output:=false \
  enable_bird_detector:=false \
  enable_bird_3d_fusion:=false

python3 scripts/waver_field_readiness_check.py --level L1 --strict
```

## 6. L2 Sensor-Only

Motors stay disabled. Verify LiDAR, scan conversion, and TF.

```bash
python3 scripts/waver_field_readiness_check.py \
  --level L2 \
  --strict \
  --scan-topic /scan_safety \
  --require-scan true
```

If `/scan_safety` is not produced yet, do not proceed to wheel checks.

## 7. L3 Wheel-Off Driver Check

Only run this with wheels lifted.

```bash
python3 scripts/waver_base_feedback_probe.py \
  --serial-port /dev/serial/by-id/<WAVER_SERIAL_ID>

python3 scripts/waver_motor_calibration_wizard.py \
  --serial-port /dev/serial/by-id/<WAVER_SERIAL_ID> \
  --allow-hardware \
  --wheel-off-confirm

python3 scripts/waver_field_readiness_check.py \
  --level L3 \
  --strict \
  --serial-port /dev/serial/by-id/<WAVER_SERIAL_ID> \
  --enable-waver-base-driver true
```

## 8. L4 Closed-Area Low-Speed Wheel-On

Wheel-on starts only after L3 PASS and the hardware acceptance matrix has
evidence for required L4 items.

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
  map:=$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml

python3 scripts/waver_field_readiness_check.py \
  --level L4 \
  --strict \
  --serial-port /dev/serial/by-id/<WAVER_SERIAL_ID>
```

Bird detector, 3D fusion, sound deterrent, target approach, gimbal, and
experiment logging remain disabled by default in the real bird launch. Enable
them only for an explicitly approved experimental profile, for example:

```bash
ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  enable_bird_detector:=true \
  enable_bird_3d_fusion:=true \
  enable_camera_gimbal_controller:=true \
  enable_target_goal_manager:=true \
  enable_target_departure_monitor:=true \
  enable_radar_command_bridge:=true \
  enable_experiment_logger:=true \
  enable_sound_deterrent:=true \
  enable_sound_output:=false \
  bird_model_path:=$HOME/models/bird_yolov8n.pt
```

## 9. Emergency Stop And Shutdown

Use this software stop helper before killing terminals:

```bash
bash scripts/waver_field_stop_all.sh
```

The physical E-stop remains the primary stop method. The helper publishes zero
velocity and stops common Waver field nodes, but it is not a substitute for a
hardware stop.

## 10. Troubleshooting

- If `/cmd_vel` has more than one publisher, stop immediately and run
  `bash src/waver_patrol/scripts/waver_cmd_chain_check.sh`.
- If `/waver/mode` has more than one publisher, stop mission testing.
- If `/voltage` is implausible, verify USB power/backfeed and battery scale.
- If odom does not update, inspect `config/waver_base_feedback_schema.yaml`
  and `/waver/base_driver_state`.
- If LiDAR is present but no scan exists, check `/livox/lidar`, `/livox/imu`,
  TF, and the scan adapter launch.
- If a field UI opens without Jetson reachability, close it. A local-only UI
  can show changing values without moving the robot.

## 11. Known Limitations

- Real camera-LiDAR extrinsics are not proven by source checks.
- Battery voltage scaling must be calibrated under load.
- `nav2_collision_monitor` is not the current final command authority.
  `safety_cmd_mux_node` is the implemented final gate.
- Full L5 bird autonomy requires detector model validation, 3D fusion
  evidence, target association evidence, sound-output acknowledgement, and
  blackbox logs.
