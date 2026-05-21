# Waver Bird Autonomy Integration

This stack is a conservative autonomy layer for Waver bird-detection patrol experiments. It is not
a complete outdoor autonomous driving product. Keep a human E-stop operator present for all real
tests.

## Command Ownership

Only `safety_cmd_mux_node` publishes final `/cmd_vel`.

| Node | Input | Output | Role |
| --- | --- | --- | --- |
| `fixed_waypoint_patrol_node` | `/odom`, waypoint yaml | `/waver/cmd_vel_patrol`, `/waver/patrol_state` | Low-speed odom-frame waypoint candidate command. |
| `lidar_aerial_motion_detector_node` | `/waver/lidar_objects`, `/lidar/detections`, `/waver/object_point` | `/waver/aerial_target`, `/waver/aerial_target_active` | Selects moving height/depth candidates from 3D-like object topics. |
| `target_body_tracker_node` | `/waver/aerial_target`, `/waver/aerial_target_active` | `/waver/cmd_vel_target_track` | Turns the rover body toward the target. ROS2 `angular.z > 0` means left turn. |
| `battery_return_manager_node` | `/battery_state`, `/voltage`, `/waver/return_home_arrived` | `/waver/return_home_active`, `/waver/return_goal` | Requests return-home on critical battery. Voltage thresholds require field calibration. |
| `return_home_controller_node` | `/waver/return_home_active`, `/waver/return_goal`, `/odom` | `/waver/cmd_vel_return_home`, `/waver/return_home_arrived` | Low-speed odom-frame home controller. |
| `auto_behavior_mux_node` | Candidate commands and mode topics | `/waver/cmd_vel_auto`, `/waver/auto_behavior_state` | Selects return, target tracking, patrol, or idle by mode priority. |
| `safety_cmd_mux_node` | auto/manual commands, `/scan`, stops, speed limit, mode | `/cmd_vel`, `/waver/safety_state` | Final E-stop, scan safety, manual override, timeout, clamp, and rate limits. |

## 2D LiDAR Limitation

`sensor_msgs/LaserScan` has range and angle only. It does not provide object height. Use it for
front hard-stop and slow-down, not for bird/drone/airborne classification. Height-aware candidates
must come from `PoseArray`, `PointStamped`, or a future `PointCloud2` adapter.

Current PointCloud2 behavior is intentionally fail-safe: the handler warns, publishes an inactive
state, and does not throw `NotImplementedError` inside a ROS callback.

## Mode Policy

| Mode | Behavior priority |
| --- | --- |
| `MANUAL` | Manual command only after safety filters. |
| `AUTO` | `BATTERY_RETURN`, `TARGET_TRACK`, `WAYPOINT_PATROL`, `IDLE`. |
| `PATROL` | `BATTERY_RETURN`, `WAYPOINT_PATROL`, `IDLE`. |
| `TRACK_ONLY` | `BATTERY_RETURN`, `TARGET_TRACK`, `IDLE`. |
| `RETURN_HOME` | `BATTERY_RETURN` only. |
| `STANDBY` | Stop automatic behavior. |
| `EMERGENCY` / `DISABLED` | Final stop. |

## Desk Test

```bash
cd /home/chotaehyun/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch waver_patrol waver_bird_autonomy.launch.py \
  use_sim_odom:=true \
  enable_test_publishers:=true \
  require_scan:=false \
  start_serial_bridge:=false
```

`require_scan:=false` and the test publishers are forbidden for real driving.

## Gazebo Test

Run Gazebo with the same safety ownership used on the real robot:

```bash
cd /home/chotaehyun/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch waver_patrol waver_gazebo_bird_autonomy.launch.py \
  use_gui:=false \
  require_scan:=true \
  enable_scripted_keyboard_test:=false
```

Expected checks:

```bash
ros2 topic info -v /cmd_vel
ros2 topic echo --once /waver/safety_state
ros2 topic echo --once /waver/auto_behavior_state
ros2 topic echo --once /waver/aerial_target_active
ros2 topic echo --once /waver/target_tracking_state
ros2 topic echo --once /waver/cmd_vel_target_track
```

Observed in this workspace:

| Check | Result |
| --- | --- |
| UGV spawned in Gazebo | Pass |
| Bird and swarm models spawned | Pass |
| `/cmd_vel` publisher count | Pass, only `safety_cmd_mux_node` |
| Safety scan status | Pass, `AUTO_PASS SCAN_CLEAR` |
| Waypoint behavior selected | Pass, `mode=PATROL selected=WAYPOINT_PATROL` |
| Bird target active | Pass |
| Target left command sign | Pass, `TARGET_LEFT_TURN_LEFT` with `angular.z > 0` |
| Emergency stop | Pass, `/cmd_vel` becomes zero |

The local Gazebo UGV model still logs that `libros2_livox.so` is missing. The Waver launch uses
the depth camera point cloud `/camera/points` for the simulated safety scan. On a system with the
Livox Gazebo plugin installed, switch the launch arguments to `/mid360` and the normal x/y/z axes.

## Real Robot Bringup Sequence

1. Lift wheels off the ground.
2. Confirm no other process owns the serial port: `app.py`, `ugv_driver`, or another bridge.
3. Launch without serial first:

```bash
ros2 launch waver_patrol waver_bird_autonomy.launch.py \
  require_scan:=true \
  start_serial_bridge:=false \
  enable_test_publishers:=false
```

4. Check `/scan`, `/odom`, `/waver/safety_state`, and `/cmd_vel`.
5. Test `/waver/emergency_stop` and `/waver/external_stop`.
6. Only then enable serial:

```bash
ros2 launch waver_patrol waver_bird_autonomy.launch.py \
  require_scan:=true \
  start_serial_bridge:=true \
  enable_test_publishers:=false
```

## Future Hardware Hooks

| Future subsystem | Current hook |
| --- | --- |
| LiDAR cluster / 3D detector | Publish `PoseArray` on `/waver/lidar_objects` or `/lidar/detections`. |
| Single target point | Publish `PointStamped` on `/waver/object_point`. |
| PointCloud2 detector | Replace the safe warning path in `lidar_aerial_motion_detector_node.py`. |
| Deep learning classifier | Publish class/confidence to `/waver/external_target_class` and `/waver/external_target_confidence`. |
| Gimbal control | Subscribe to `/waver/aerial_target` and publish a separate gimbal command topic, not `/cmd_vel`. |
| Sound deterrent | Extend `sound_alert_stub.py`; keep `enable_sound_output=false` until bench-tested. |
| Nav2 return/patrol | Replace the stub methods in patrol and return-home controllers with action clients. |

## Real-Test Safety Checklist

1. Wheels lifted: verify `/cmd_vel` direction and stop.
2. Confirm only one final command path publishes `/cmd_vel`.
3. Confirm `safety_cmd_mux_node` is running.
4. Confirm `serial_cmd_vel_bridge` and Waveshare `ugv_driver` are not both using the serial port.
5. Confirm `/scan` is fresh before driving with `require_scan:=true`.
6. Confirm front obstacle causes `SCAN_HARD_STOP`.
7. Confirm `/odom` frame and waypoint frame match.
8. Confirm test publishers are off.
9. Confirm `enable_sound_output=false`.
10. Start at low speed in a controlled, empty area with a physical stop method.
