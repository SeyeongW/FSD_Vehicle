# Waver Operator Modes

This page defines the three operator-facing modes that should be used before
real field work. The commands are examples for `~/ugv_ws/FSD_Vehicle`; adjust
only topic names and device IDs that are physically verified.

## Mode Table

| mode | launch/script | fake allowed? | serial allowed? | sound output? | expected `/cmd_vel` owner | required checks |
| --- | --- | --- | --- | --- | --- | --- |
| Gazebo pipeline validation | `ros2 launch waver_patrol waver_gazebo_bird_autonomy.launch.py use_sim_time:=true enable_fake_sound:=true enable_trial_logger:=true` | yes, clearly SIM_ONLY | no | fake state only | `safety_cmd_mux_node` | quality gate, Gazebo topic graph, log evidence level L2 |
| Real sensor-only dry run | `ros2 launch waver_patrol waver_real_bird_autonomy.launch.py start_serial_bridge:=false enable_waver_base_driver:=false default_mode:=STANDBY enable_sound_output:=false` | no | no | no | `safety_cmd_mux_node`, zero output | strict preflight minus wheel motion, detector/sensor states |
| Wheel-off / low-speed wheel-on | `ros2 launch waver_patrol waver_real_bird_autonomy.launch.py enable_waver_base_driver:=true serial_port:=/dev/serial/by-id/<WAVER_SERIAL_ID> default_mode:=STANDBY safety_max_linear_speed:=0.05 safety_max_angular_speed:=0.20 enable_sound_output:=false` | no | yes, one owner | no unless separately approved | `safety_cmd_mux_node` | `waver_real_preflight_check.sh --strict --wheel-on`, physical E-stop, wheels raised first |

## Gazebo Validation Command

```bash
cd ~/ugv_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch waver_patrol waver_gazebo_bird_autonomy.launch.py \
  use_sim_time:=true \
  enable_fake_sound:=true \
  enable_trial_logger:=true
```

Gazebo synthetic success is pipeline validation. It must not be described as
real YOLO accuracy, real deterrence performance, or outdoor field proof.

## Real Sensor-Only Dry Run

```bash
cd ~/ugv_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  start_serial_bridge:=false \
  enable_waver_base_driver:=false \
  default_mode:=STANDBY \
  enable_sound_output:=false \
  bird_model_path:=/absolute/path/to/model.pt
```

The robot must remain stopped. The expected result is valid sensor, TF,
detector, and safety state reporting with zero final `/cmd_vel`.

## Wheel-Off / Low-Speed Wheel-On

Use `/dev/serial/by-id/...` for the serial port. Avoid `/dev/ttyUSB0` in field
procedures because enumeration can change after reconnects.

```bash
cd ~/ugv_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/waver_patrol/scripts/waver_real_preflight_check.sh --strict --wheel-on
ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  enable_waver_base_driver:=true \
  serial_port:=/dev/serial/by-id/<WAVER_SERIAL_ID> \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  enable_sound_output:=false
```

Start wheel-off first. Move to wheel-on only after serial direction, braking,
E-stop, stale command timeout, and obstacle stop behavior have been verified.
