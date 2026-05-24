# Waver Gazebo 실행 가이드

## Gazebo 실행

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=true \
  use_operator_panel:=false \
  world_file:=$HOME/ros2_ws/FSD_Vehicle/src/ugv_main/ugv_gazebo/worlds/ugv_world.world \
  robot_sdf_file:=$HOME/ros2_ws/FSD_Vehicle/src/ugv_main/ugv_gazebo/models/ugv_rover/model.sdf \
  robot_spawn_x:=0.0 \
  robot_spawn_y:=0.0 \
  robot_spawn_z:=0.15
```

---

## 리모콘 실행

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_topic:=/map \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  require_scan:=false \
  map_display_mode:=map_fixed \
  publish_direct_cmd_vel:=false
```

---

## 토픽 및 노드 확인

### 토픽 목록 확인

```bash
ros2 topic list
```

### 노드 목록 확인

```bash
ros2 node list
```

---

## 주요 토픽 Hz 확인

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
ros2 topic hz /amcl_pose
```

---

## Mid360 LiDAR PointCloud 확인

### PointCloud2 Hz 확인

```bash
ros2 topic hz /mid360_PointCloud2
```

### PointCloud2 데이터 1회 확인

```bash
ros2 topic echo /mid360_PointCloud2 --once
```

---

## Mission 상태 확인

```bash
ros2 topic echo /waver/mission_state
ros2 topic echo /waver/safety_state
ros2 topic echo /waver/current_waypoint
ros2 topic echo /waver/active_nav_goal
```

---

## LiDAR Object 관련 토픽 확인

```bash
ros2 topic echo /waver/lidar_objects
ros2 topic echo /waver/lidar_objects_map
ros2 topic echo /waver/elevated_dynamic_targets
ros2 topic echo /waver/moving_target_valid
```

---

## Nav2 Path 확인

### Global Path 확인

```bash
ros2 topic echo /plan
```

### Local Path 확인

```bash
ros2 topic echo /local_plan
```

### Mapping Node ㅎ확인

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=131

python3 src/waver_patrol/scripts/run_mapping_mode_check.py \
  --output-root ~/ros2_ws/experiments_result \
  --experiment-name mapping_mode_manual \
  --duration-sec 14 \
  --save-after-sec 9
```
  
