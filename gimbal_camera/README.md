# gimbal_camera

Standalone 3-axis gimbal camera for **Gazebo Sim (Harmonic / gz-sim8)**, based on
the PX4 CGO3 gimbal. The joints are driven by gz's `JointPositionController`
(the PID runs *inside* gz at the physics rate), so you command **target angles**
and the camera holds them rock-steady — no tremor.

## Layout
```
gimbal_camera/
├── models/
│   ├── gimbal/          # PX4 CGO3 gimbal (model.sdf + meshes) — the active one
│   └── siyi_a8_mini/    # earlier hand-rolled attempt (unused, kept for reference)
├── worlds/gimbal.sdf    # empty world + Sensors system, includes model://gimbal
├── launch/gimbal_camera.launch.py
└── scripts/gimbal_keyboard_control.py
```

## Requirements
ROS 2 Humble + the ros_gz packages (already installed on this machine):
`ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`. No `colcon build` needed — the
launch file resolves its own paths.

```bash
source /opt/ros/humble/setup.bash
```

## Run
```bash
# Terminal 1 — simulator + bridges
ros2 launch ~/gimbal_camera/launch/gimbal_camera.launch.py

# Terminal 2 — keyboard control (must be a real terminal)
~/gimbal_camera/scripts/gimbal_keyboard_control.py
```

Keys: `a`/`d` yaw · `w`/`s` pitch · `z`/`c` roll · `space` reset · `q` quit
(0.05 rad per keypress).

## Topics
| topic | type | dir |
|---|---|---|
| `/gimbal/yaw_cmd` `/gimbal/pitch_cmd` `/gimbal/roll_cmd` | `std_msgs/Float64` | target angle [rad] |
| `/gimbal/joint_states` | `sensor_msgs/JointState` | feedback |
| `/gimbal/camera` | `sensor_msgs/Image` | camera |
| `/gimbal/camera_info` | `sensor_msgs/CameraInfo` | camera |

Command an angle without the keyboard:
```bash
ros2 topic pub -1 /gimbal/pitch_cmd std_msgs/msg/Float64 "{data: -0.8}"
```
View the camera:
```bash
ros2 run rqt_image_view rqt_image_view /gimbal/camera
```

## Joint ranges
- yaw (`cgo3_vertical_arm_joint`): continuous
- roll (`cgo3_horizontal_arm_joint`): ±45°
- pitch (`cgo3_camera_joint`): −135° … +45°

## Notes
- The launch forces `gz_version:=8` (Harmonic). Without it, ros_gz defaults to
  Fortress (`ign gazebo`) which can't read this world.
- Gimbal PID gains live in `models/gimbal/model.sdf` (the three
  `JointPositionController` plugins), not in the Python node.
