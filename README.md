# Waver ROS2 Humble Final Check Guide

Workspace for this release:

```bash
WORKSPACE_ROOT=$HOME/ros2_ws5/FSD_Vehicle
cd $WORKSPACE_ROOT
git branch --show-current   # must be jo
```

This tree is configured for Gazebo Classic mapping debug, Gazebo bird-autonomy
simulation, real-vehicle dry-run, wheel-off, and first low-speed wheel-on checks.
Real wheel-on is not allowed until strict preflight and wheel-off checks pass.

## Build And Static Checks

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash

bash src/waver_patrol/scripts/waver_duplicate_package_check.sh

python3 -m compileall -q src/waver_patrol src/ugv_main/ugv_tools src/ugv_main/ugv_bringup

python3 - <<'PY'
import pathlib, yaml
for p in pathlib.Path("src/waver_patrol/config").glob("*.yaml"):
    with p.open() as f:
        yaml.safe_load(f)
    print("YAML_OK", p)
PY

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Clean graph before every repeated trial:

```bash
bash src/waver_patrol/scripts/waver_cleanup_stale_nodes.sh
bash src/waver_patrol/scripts/waver_clean_graph_check.sh
```

## Gazebo Mapping Debug

Terminal 1:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_gazebo_mapping_debug.launch.py \
  use_sim_time:=true \
  use_gui:=true \
  use_operator_panel:=true \
  mapping_backend:=scan_mapper \
  scan_source_slam:=gazebo_laser \
  scan_source_safety:=gazebo_laser \
  save_dir:=$HOME/ros2_ws5/FSD_Vehicle/maps
```

Terminal 2:

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

bash src/waver_patrol/scripts/waver_scan_quality_check.sh /scan_slam
bash src/waver_patrol/scripts/waver_mapping_health_check.sh
ros2 topic info -v /waver/mode
ros2 topic info -v /cmd_vel
ros2 topic info -v /map
```

In the remote panel, use `SLAM MAPPING`, drive with WASD, then `SAVE MAP` and
`APPLY FIXED MAP`. The UI should switch to `SLAM_LIVE` during mapping and only
show `MAP_FIXED_READY` after apply.

Map quality check:

```bash
python3 src/waver_patrol/scripts/waver_map_quality_check.py \
  ~/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml
```

## Gazebo Bird Autonomy Simulation

This uses `ugv_gazebo/worlds/ugv_world.world`, `ugv_rover`, Mid-360 pointcloud,
camera topics, bird models, Gazebo-only classification/sound stubs, and the
safety mux command chain.

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_gazebo_bird_autonomy.launch.py \
  use_gui:=true \
  use_rviz:=true \
  use_sim_time:=true \
  world:=ugv_world.world \
  pointcloud_topic:=/mid360_PointCloud2 \
  camera_image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  enable_bird_detector:=true \
  enable_bird_3d_fusion:=true \
  enable_fake_sound:=true \
  enable_trial_logger:=true
```

Start patrol from UI or:

```bash
ros2 topic pub --once /waver/mission_command std_msgs/msg/String "{data: 'START_PATROL'}"
```

Expected mechanism:

```text
START_PATROL -> waypoint patrol -> Mid-360 dynamic candidate -> tracking/lock
-> target-relative offset goal -> camera alignment/classification
-> bird-confirmed-only simulated sound -> response logging -> return to patrol
```

Gazebo-only fake/stub publishers are allowed here, but real launch blocks them.

## Three-Trial Gazebo Run

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

for i in 1 2 3; do
  bash src/waver_patrol/scripts/waver_cleanup_stale_nodes.sh
  bash src/waver_patrol/scripts/waver_clean_graph_check.sh
  ros2 launch waver_patrol gazebo_moving_object_trial.launch.py \
    use_sim_time:=true \
    use_gui:=true \
    world:=ugv_world.world \
    enable_mission_stack:=true \
    enable_cluster_node:=true \
    enable_trial_logger:=true \
    enable_fake_camera_classification:=true \
    enable_fake_sound:=true \
    default_mode:=AUTO \
    trial_id:=trial_00${i}
done
```

For paper data, keep raw logs, CSV files, maps, screenshots, and rosbag outputs
under `~/ros2_ws5/FSD_Vehicle/experiments_result`.

## Topic Authority Checks

```bash
ros2 topic info -v /cmd_vel       # exactly one publisher: safety_cmd_mux_node
ros2 topic info -v /waver/mode    # exactly one publisher: mission_patrol_manager_node
ros2 topic info -v /scan
ros2 topic info -v /scan_slam
ros2 topic info -v /scan_safety
ros2 topic info -v /map
bash src/waver_patrol/scripts/waver_cmd_chain_check.sh
```

## Bird Detector/Fusion Checks

```bash
ros2 topic echo /waver/bird_detector_state
ros2 topic echo /waver/bird_fusion_state
ros2 topic echo /waver/bird_confirmed
ros2 topic echo /waver/bird_target_valid
ros2 topic echo /waver/target_class
ros2 topic echo /waver/target_confidence
ros2 topic echo /waver/sound_alert_state
```

Camera-only detections must not create navigation goals. Target approach requires
LiDAR dynamic candidate or 3D fusion validity. Sound is blocked until
`bird_confirmed=true` and safety is clear.

## Real Vehicle Dry-Run

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  start_serial_bridge:=false \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  scan_source_safety:=mid360 \
  scan_source_slam:=mid360 \
  odom_source:=ekf \
  map:=$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml \
  bird_model_path:=$HOME/models/bird_yolo.pt
```

Dry-run must show:

```text
/cmd_vel publisher count == 1
/waver/mode publisher count == 1
STANDBY keeps /cmd_vel zero
no fake/test/Gazebo-only nodes
deep_learning_bridge_stub is absent
camera-only goal generation is blocked
```

## Wheel-Off

Only run with the robot lifted or drive wheels disconnected.

```bash
cd ~/ros2_ws5/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

bash src/waver_patrol/scripts/waver_real_preflight_check.sh --strict --wheel-on

ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  start_serial_bridge:=true \
  serial_port:=/dev/serial/by-id/<WAVER_SERIAL_ID> \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  map:=$HOME/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml \
  bird_model_path:=$HOME/models/bird_yolo.pt
```

Check WASD path:

```text
waver_remote_panel -> /waver/manual_cmd_vel -> safety_cmd_mux_node
-> /cmd_vel -> serial/base driver
```

Key release, timeout, STOP, and E-STOP must all produce zero command.

## Wheel-On Low Speed

Wheel-on is allowed only after:

```text
dry-run PASS
strict preflight PASS
wheel-off PASS
physical E-STOP ready
/cmd_vel publisher count == 1
/waver/mode publisher count == 1
/scan or /scan_safety, /odom, and TF are healthy
serial direction and braking are verified
```

First wheel-on speed cap:

```text
linear <= 0.05 m/s
angular <= 0.20 rad/s
```

Stop immediately on any safety fault, duplicate publisher, stale sensor,
uncontrolled motion, or unexpected sound event.
