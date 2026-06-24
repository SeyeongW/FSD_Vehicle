# Waver Real Vehicle Test Plan

This plan is for manual execution only. Automated Codex/CI runs must stay in
hardware-free mode.

## 0. Local/Jetson Field Control Location

The field workflow is intentionally kept separate from the hardware-free
quality gate:

```text
Local PC UI -> SSH -> Jetson host -> docker exec fsd_dev_jetson -> ROS2 nodes -> Waver USB serial
```

Jetson host defaults are configured in:

```text
scripts/waver_field_docker_backend_start.sh
scripts/waver_field_local_ui_start.sh
```

Runtime override:

```bash
JETSON_HOST=<current_jetson_ip> bash scripts/waver_field_docker_backend_start.sh
JETSON_HOST=<current_jetson_ip> bash scripts/waver_field_local_ui_start.sh
```

Do not change this workflow during package convergence unless the operator
explicitly asks for field-control work.

## 1. Bench Preflight

```bash
cd ~/ugv_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
bash scripts/waver_quality_gate.sh --no-hardware --skip-build --skip-colcon-test
```

Confirm:

- branch is `jo`
- final `/cmd_vel` owner is safety mux in launch/config
- real launch defaults to `STANDBY`
- real launch does not start gazebo, fake, test, or stub nodes
- serial owner count is designed to be one

## 2. Sensor-Only Dry Run

Run without serial command output:

```bash
ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  start_serial_bridge:=false \
  enable_waver_base_driver:=false \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20
```

Confirm:

- `/cmd_vel` remains zero in STANDBY
- `/scan` or `/scan_safety` is fresh when LiDAR is connected
- `/odom` and `odom -> base_link` are available before autonomy
- bird detector reports safe false when model/camera is missing

## 3. Wheel-Off Test

Lift the rover or remove wheel contact. Keep physical E-stop reachable.

```bash
bash src/waver_patrol/scripts/waver_real_preflight_check.sh --strict --wheel-on
```

Then start the field backend and local UI using the validated SSH/Docker bridge
scripts. Test `W`, `S`, `A`, `D`, release timeout, STOP, E-stop, and serial
reconnect stop burst.

## 4. Low-Speed Wheel-On Test

Only after wheel-off passes:

- linear speed <= 0.05 m/s
- angular speed <= 0.20 rad/s
- clear area
- physical E-stop operator present
- one serial owner
- one final `/cmd_vel` publisher

Test order:

1. 0.1 m forward and stop.
2. 0.1 m reverse and stop.
3. Small left and right pivot.
4. 0.3 m square patrol.
5. Nav2 1 m goal with obstacle-free path.
6. Bird-detection dry run without sound output.
7. Sound output only after legal/safety ack.

## 5. Feedback Into Regression

For every field test, record:

- exact command lines
- branch and commit
- sensor topics
- `/cmd_vel`, `/waver/manual_cmd_vel`, `/waver/safety_state`
- `/waver/mode`, `/waver/mission_state`
- `/odom`, `/tf`, `/voltage`
- operator observations

Store reports under `reports/hardware_feedback/` and rosbags under a clearly
named external archive. Then use `scripts/waver_rosbag_replay_check.sh` to add
hardware-free regression coverage.
