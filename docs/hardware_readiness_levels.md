# Waver Hardware Readiness Levels

This document defines fail-closed readiness levels for running Waver on real
hardware. Automated tests must never move real wheels, open speakers, toggle
GPIO outputs, or claim wheel-on readiness without explicit field evidence.

| Level | Purpose | Required topics | Required nodes | Forbidden nodes | Required env/manual ack | Pass/fail criteria |
|---|---|---|---|---|---|---|
| L0_SOURCE_CHECK | Source/static/package readiness with no hardware access | none | none | real serial, sound, GPIO, Gazebo-only publishers in real profile | `WAVER_NO_HARDWARE=1` recommended | compile, shell syntax, no-ROS/unit/static/contract checks pass |
| L1_ROS_GRAPH_DRY_RUN | Bring up real-profile ROS graph with serial and sound disabled | `/waver/mode`, `/waver/safety_state` when graph is running | `mission_patrol_manager_node`, `safety_cmd_mux_node` | fake/test/Gazebo/mock nodes, serial bridge, base driver, sound output | serial disabled, sound disabled | graph starts without actuator subscriber; no fake/test nodes |
| L2_SENSOR_ONLY_LIVE | Verify live sensors with motors disabled | `/scan` or `/scan_safety`, `/tf`, optional pointcloud/camera | LiDAR driver or adapter, TF publishers | `/cmd_vel` actuator subscriber, base driver, serial bridge, sound output | motor serial disabled | scan rate and TF checks pass; no actuator subscriber exists |
| L3_WHEEL_OFF_DRIVER | Verify base driver feedback with wheels physically off ground | `/odom` or `/odom_raw`, `/imu/data_raw`, `/voltage`, `/waver/base_driver_state`, `/waver/serial_owner_state` | `waver_base_driver_node`, `safety_cmd_mux_node` | autonomous patrol, Nav2 mission start, fake/test drivers | `--wheel-off-confirm`, by-id serial port | serial owner single, odom/IMU/voltage/base feedback fresh; tiny manual command only |
| L4_WHEEL_ON_LOW_SPEED | Closed-area supervised low-speed wheel-on check | L3 topics plus `/scan` or `/scan_safety`, `/waver/safety_state`, `/cmd_vel` | base driver, safety mux, localization/Nav2 as configured | fake/test/Gazebo/mock nodes, bird/sound experimental stack by default | human E-stop, closed area, max linear <= 0.05, max angular <= 0.20 | strict scan/odom/TF/battery/safety/cmd-chain pass; one goal or one waypoint only |
| L5_AUTONOMOUS_PATROL | Guarded autonomous patrol after L4 evidence | L4 topics plus blackbox recording and mission state | patrol manager, safety mux, battery return, obstacle stop, blackbox recorder | sound/bird experimental stack unless explicitly selected and acknowledged | accumulated L4 evidence, blackbox logging, E-stop evidence | patrol, obstacle stop, battery return, emergency stop, and logging evidence pass |

## Common Required Checks

- Final `/cmd_vel` publisher must be exactly one node:
  `safety_cmd_mux_node`. `collision_monitor` may be accepted only in a future
  explicitly documented topology with `WAVER_ALLOW_COLLISION_MONITOR_FINAL=1`
  and a dedicated command-chain verification.
- Real actuator subscriber must be exactly one only when the level allows motor
  hardware.
- Real profile must not start fake detector, test publishers, Gazebo-only nodes,
  or mock backends.
- Serial port must be `/dev/serial/by-id/...` for L3 and above.
- Sound output must remain disabled unless hardware and legal acknowledgements
  are explicitly provided.
- `FIELD_READINESS=PASS` is allowed only when every required check for the
  selected level passes.

## Collision Monitor Policy

This hardware release uses `safety_cmd_mux_node` as the implemented safety
command gate. `nav2_collision_monitor` is treated as a future optional
enhancement until it is wired into the real command chain and verified by the
readiness checker. Do not claim collision-monitor safety for this release.
