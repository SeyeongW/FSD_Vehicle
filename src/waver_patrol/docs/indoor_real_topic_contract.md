# Indoor Real Topic Contract

This contract applies to `waver_indoor_patrol_real.launch.py` and first indoor low-speed Waver tests.

## Command Chain

```text
Nav2 controller
  -> /waver/cmd_vel_nav2_raw
  -> nav2_velocity_smoother
  -> /waver/cmd_vel_nav2_smooth
  -> safety_cmd_mux_node
  -> /cmd_vel
  -> waver_base_driver_node or the selected single canonical serial bridge
```

When the indoor launch disables the velocity smoother:

```text
Nav2 controller
  -> /waver/cmd_vel_nav2
  -> safety_cmd_mux_node
  -> /cmd_vel
```

Manual operator control:

```text
waver_remote_panel
  -> /waver/manual_cmd_vel
  -> safety_cmd_mux_node
  -> /cmd_vel
```

## Required Topic Owners

| Topic | Required Owner |
| --- | --- |
| `/cmd_vel` | `safety_cmd_mux_node` only |
| `/waver/mode` | `mission_patrol_manager_node` only |
| `/scan` or `/scan_safety` | exactly one active scan publisher |
| `/odom` | exactly one odometry/EKF publisher |

## Mission Topics

| Topic | Direction | Notes |
| --- | --- | --- |
| `/waver/mode_cmd` | UI/operator -> mission manager | Request only; not state authority. |
| `/waver/mission_command` | UI/operator -> mission manager | `START_PATROL`, `STOP`, emergency clear requests. |
| `/waver/mode` | mission manager -> system | State topic. Only one publisher. |
| `/waver/mission_state` | mission manager -> UI/logging | Read-only status. |

## Safety Topics

| Topic | Notes |
| --- | --- |
| `/waver/safety_state` | Must treat `STANDBY_STOP`, `AUTO_COMMAND_TIMEOUT_STOP`, and `MANUAL_COMMAND_TIMEOUT_STOP` as normal no-command states. |
| `/waver/emergency_stop` | Bool. True forces final zero command. |
| `/waver/external_stop` | Bool. True forces final zero command. |
| `/waver/livox_scan_adapter_state` | `OK_CLEAR` and `OK_OBSTACLE` are allowed states; `DEGRADED`, `STALE`, `FAILED`, `NO_POINTS`, `EMPTY`, `NOT_ENABLED` are strict-fail states. |
| `/waver/base_driver_state` | Base-driver serial and command state. |
| `/waver/serial_owner_state` | Should show one serial owner. |
| `/waver/battery_safety_state` | Hard gate only when battery requirement is enabled. |

## Scan And Odom Contract

- Scan topic defaults to `/scan`.
- Livox pointcloud defaults to `/livox/lidar`.
- Scan quality check must report Hz, finite count, front min, rear min, and adapter state.
- Real navigation requires `map -> odom -> base_link`.
- Wheel-off must confirm positive `linear.x` equals physical forward motion.
- Wheel-off must confirm positive `angular.z` direction matches operator convention.

## Forbidden In Indoor Real Default

- Fake/test/Gazebo publishers.
- `deep_learning_bridge_stub`.
- Direct `/cmd_vel` publication from Nav2, UI, teleop, target manager, or driver node.
- Camera-only bird detection creating a navigation goal.
- Collision Monitor as final `/cmd_vel` owner unless explicitly enabled in a future profile and tested separately.

## Rosbag Record List

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

