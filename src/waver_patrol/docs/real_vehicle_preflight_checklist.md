# Real Vehicle Preflight Checklist

Run this before any wheel-on autonomous test:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/waver_patrol/scripts/waver_real_preflight_check.sh
```

## Hard Gates

- Gazebo H1/H2/H3 height-based trials passed.
- `build/`, `install/`, `log/`, `bags/`, and `experiments_result*` are not committed.
- `start_serial_bridge:=false` until wheel-off direction tests pass.
- `enable_sound_output:=false`.
- test publishers are off.
- real first speed limit is `<= 0.1 m/s`.
- physical E-stop is available.
- final `/cmd_vel` publisher count is one.

## Command Ownership

```bash
ros2 topic info -v /cmd_vel
```

Expected:

- publisher: `safety_cmd_mux_node`
- Nav2 candidate input: `/waver/cmd_vel_nav2`
- manual candidate input: `/waver/manual_cmd_vel`

Do not run `ugv_driver`, old serial bridges, keyboard teleop, and Waver serial bridge together.

## Sensor And TF

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic list | grep -E 'PointCloud2|points|livox|mid360|lidar_objects'
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link lidar_frame
```

Height target detection requires a real z source:

- 3D LiDAR `sensor_msgs/msg/PointCloud2`,
- depth/stereo camera,
- custom 3D detection message.

2D LaserScan alone must be treated as `z_valid=false`.

## Mapping And Patrol

- Mapping Mode: target mission trigger disabled.
- Save map with `nav2_map_server map_saver_cli`.
- Patrol Mode: map loaded, localization active, operator panel shows `MAP_FIXED`.
- Robot yaw changes must rotate only the robot arrow, not the map image.

## Mission Gate

Target is valid only when:

```text
object_height_m >= 3.0
z_valid == true
dynamic_filter_pass == true
ego_motion_compensated == true
```

Static high objects, low dynamic objects, and z-unknown objects must not trigger target navigation.
