# Current Waver Field Status Prompt

Use this as the starting context for the next Waver 실차 debugging session.

- Workspace: `~/ros2_ws2/FSD_Vehicle`
- Branch: `jo`
- Jetson: `sw@10.139.225.150`
- ROS: Humble, `ROS_DOMAIN_ID=30`, Jetson uses `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Local PC runs `waver_remote_panel`; Jetson runs backend/safety/base driver.

Known-good state:

- Current working transport is Jetson USB serial, not 40-pin UART.
- Field power/USB order that avoided the low-voltage display issue:
  1. Open SSH from local PC to Jetson first: `ssh sw@10.139.225.150`.
  2. Power/connect Waver normally and confirm the Waver/OLED voltage is normal.
  3. Connect the Jetson USB port to the Waver USB serial port after the above two are stable.
  4. Then start the Docker backend and local PC remote panel.
  - If USB is connected before the Waver/Jetson power state is stable, the Waver OLED may show about `0.6V` due to USB VBUS/back-power/partial-power behavior. Do not run wheel-on tests in that state.
- The active serial path is:
  - `/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_74d1434bde00f011b2efc1295c2a50c9-if00-port0`
  - symlink target `/dev/ttyUSB0`
- The Docker backend must run `waver_base_driver_node` on that USB by-id path.
- Do not leave the backend on `/dev/ttyTHS1` or `/dev/ttyTHS2`; those 40-pin UART probes produced no Rover feedback.
- Motor power issue was fixed by enabling/checking Waver motor power.
- `/waver/base_driver_state` now reports `motor_power=OK`.
- Voltage observed after fix: about `8.3V`, later about `8.0V`.
- `/cmd_vel` publisher count is 1: `safety_cmd_mux_node`.
- `/waver/mode` publisher count is 1: `mission_patrol_manager_node`.
- Serial owner is 1: `waver_base_driver_node`.
- WAVE ROVER command protocol is `lr`, not `t13`.
- Real serial payload for forward command:
  - `{'T': 1, 'L': 0.25, 'R': 0.25}`
- Verified after USB reconnection:
  - UI W key path produced `payload={'T': 1, 'L': 0.25, 'R': 0.25}` through `waver_base_driver_node`.
  - UI START_PATROL/P key produced patrol states `ARRIVED`, `ALIGN_HEADING`, `CRUISE` and base payloads:
    - forward: `{'T': 1, 'L': 0.25, 'R': 0.25}`
    - slow turn: `{'T': 1, 'L': 0.0, 'R': 0.08}`
  - Final STOP/STANDBY returned `/cmd_vel` and base payload to zero.
- Latest verified local-PC UI test after responsiveness retune:
  - Local UI must be started from local PC, not inside the Jetson Docker container.
  - Tk/X11 synthetic test requires `focus -force .` before `event generate`; otherwise generated key events can be ignored even though the UI is open.
  - Earlier pivot turn was replaced with true counter-rotation because Waver needs in-place direction change.
  - A/left in-place turn verified through UI -> SSH bridge -> safety mux -> base driver:
    - `payload={'T': 1, 'L': -0.08, 'R': 0.08}`
  - D/right in-place turn verified through UI -> SSH bridge -> safety mux -> base driver:
    - `payload={'T': 1, 'L': 0.08, 'R': -0.08}`
  - UI P/START_PATROL verified for the current 0.5 m low-speed patrol setup:
    - state: `ARRIVED: open_loop_step_complete`
    - state: `ALIGN_HEADING: open_loop_square_turn`
    - forward payload after slowing: `{'T': 1, 'L': 0.18, 'R': 0.18}`
    - turn payload target after slowing: `{'T': 1, 'L': -0.08, 'R': 0.08}`
    - STOP/STANDBY cleanup returned `{'T': 1, 'L': 0.0, 'R': 0.0}`
  - ROS `/voltage` may still be absent or base state may show `voltage_v=0.000 motor_power=UNKNOWN` when Rover feedback is not streaming, but command writes over USB serial are working. Use the Waver/OLED physical voltage as the immediate wheel-on safety check until feedback parsing is fixed.
- Local PC UI path is:
  - `waver_remote_panel`
  - SSH bridge
  - `/waver/manual_cmd_vel`
  - `safety_cmd_mux_node`
  - `/cmd_vel`
  - `waver_base_driver_node`
  - Waver serial port

Important correction:

- `T:13 X/Z` is not the active WAVE ROVER drive protocol for this vehicle.
- Keep `waver_base_driver_node` on `command_protocol:=lr`.

Current tuning:

- Manual forward works.
- Motor voltage is now OK.
- WAVE ROVER does not steer wheel angles. It uses left/right differential wheel speed.
- Official Waveshare-style movement command is JSON `{"T":1,"L":...,"R":...}`:
  - W/forward: `L > 0`, `R > 0`, expected payload about `{'T': 1, 'L': 0.25, 'R': 0.25}`.
  - A/left turn: left side stopped or slower, right side forward, expected payload about `{'T': 1, 'L': 0.0, 'R': 0.10~0.12}`.
  - D/right turn: left side forward, right side stopped or slower, expected payload about `{'T': 1, 'L': 0.10~0.12, 'R': 0.0}`.
  - W+A/W+D are inside-brake arc turns, not steering-angle turns.
- Current low-speed tuning target:
  - UI command rate: `45 Hz`.
  - Safety mux timer: `60 Hz`.
  - Base driver command rate: `60 Hz`.
  - Pure A/D turn ratio: `0.08` in counter-rotation mode; do not return to the old high turn values that caused excessive spin.
  - Mixed/diagonal outer ratio: about `0.12`, inner side `0.0`.
- Current real low-speed patrol target:
  - Open-loop square patrol.
  - Four sides, `0.5 m` per side.
  - Speed `0.03 m/s`, turn angular command `0.025 rad/s`.
  - Repeats continuously until STOP/STANDBY.
  - This is supervised wheel-on low-speed testing only. It is not encoder-accurate waypoint autonomy until odom feedback is valid.

Current backend launch values:

- `safety_cmd_mux_node`
  - `timer_hz:=60.0`
  - `manual_override_timeout_sec:=0.30`
  - `command_timeout_sec:=0.35`
  - `max_linear_delta_per_tick:=0.04`
  - `max_angular_delta_per_tick:=0.035`
- `waver_base_driver_node`
  - `command_protocol:=lr`
  - `command_rate_hz:=60.0`
  - `cmd_timeout_s:=0.20`
  - `max_left_right:=0.24`
  - `max_demo_speed:=0.24`
  - `wheel_delta_per_tick:=0.06`
  - `pure_turn_mode:=pivot`
  - `pure_turn_min_ratio:=0.24`
  - `pure_turn_max_ratio:=0.24`
  - Pure turn policy: A/left drives the right wheels only, D/right drives the left wheels only.
  - `mixed_turn_mode:=inside_brake`
  - `mixed_turn_inner_ratio:=0.0`
  - `mixed_turn_outer_ratio:=0.12`
- `waver_gazebo_patrol` for current field micro-patrol
  - `allow_open_loop_without_odom:=true`
  - `waypoint_file:=/ros2_ws/ugv_ws/src/ugv_main/ugv_tools/waypoints/waver_0p3m_patrol.yaml`
  - `open_loop_step_distance_m:=0.3`
  - `open_loop_sides_per_loop:=4`
  - `open_loop_turn_duration_s:=4.7`
  - `open_loop_turn_angular_speed:=0.025`
  - `loop_count:=-1`
  - `max_linear_speed:=0.03`
  - `max_angular_speed:=0.025`

Safe launch pattern for local PC UI:

```bash
cd ~/ros2_ws2/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30
unset RMW_IMPLEMENTATION

ros2 run ugv_tools waver_remote_panel --ros-args \
  --params-file src/ugv_main/ugv_tools/config/waver_4wd_control.yaml \
  -p publish_direct_cmd_vel:=false \
  -p profile:=operator_bridge \
  -p remote_bridge_enabled:=true \
  -p remote_bridge_host:=10.139.225.150 \
  -p remote_bridge_user:=sw \
  -p remote_bridge_password:=12341234 \
  -p remote_bridge_workspace:=/home/sw/ros2_ws2/FSD_Vehicle \
  -p remote_bridge_ros_domain_id:=30 \
  -p lidar_required:=false \
  -p enable_scan_assist:=false \
  -p allow_start_without_map:=true \
  -p allow_start_without_localization:=true \
  -p publish_mode_heartbeat:=false \
  -p command_rate_hz:=45.0 \
  -p default_speed:=0.06 \
  -p default_angular:=0.08 \
  -p max_linear_speed:=0.08 \
  -p max_angular_speed:=0.08 \
  -p max_linear_accel:=1.20 \
  -p max_angular_accel:=1.20
```

Quick checks:

```bash
ros2 topic echo --once --full-length /waver/base_driver_state
ros2 topic info -v /cmd_vel
ros2 topic info -v /waver/mode
```

Expected:

- `protocol=lr`
- `port=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_...`
- `motor_power=OK`
- `/cmd_vel` publisher: `safety_cmd_mux_node`
- `/waver/mode` publisher: `mission_patrol_manager_node`
