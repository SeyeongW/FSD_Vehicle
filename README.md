# Waver Gazebo / Real-Vehicle Run Guide

이 문서는 `jo` 브랜치 최신 구조 기준 실행 가이드다.

- 활성 소스 트리: `~/ros2_ws/FSD_Vehicle`
- 빌드/실행 기준 workspace: `~/ros2_ws/FSD_Vehicle`
- Gazebo world: `ugv_gazebo/worlds/ugv_world.world`
- Gazebo robot model: `ugv_gazebo/models/ugv_rover/model.sdf`
- SLAM Mapping 기본값: LiDAR-only `slam_gmapping` on `/scan`
- 리모콘 UI 직접 `/cmd_vel` 발행 금지: `publish_direct_cmd_vel:=false`
- 최종 `/cmd_vel` publisher: `safety_cmd_mux_node` 1개만 허용

현재 사용자 실험 기준은 `~/ros2_ws/FSD_Vehicle`이다. 이 폴더 안에서
`colcon build`, Gazebo, 리모콘 UI, SLAM mapping, 실차 dry-run을 실행한다.

---

## 0. 공통 준비

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
colcon build --packages-select ugv_tools waver_patrol ugv_slam ugv_gazebo --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=30
```

전체 빌드를 확인하려면 다음을 사용한다.

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 1. Gazebo만 실행

터미널 1에서 공항맵과 `ugv_rover`만 실행한다. 이 명령은 리모콘 UI를 같이 띄우지 않는다.

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=true \
  use_operator_panel:=false \
  robot_spawn_x:=0.0 \
  robot_spawn_y:=0.0 \
  robot_spawn_z:=0.15
```

명시적으로 world/model 경로를 지정하고 싶으면 다음처럼 설치된 package share 경로를 쓴다.

```bash
ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=true \
  use_operator_panel:=false \
  world_file:=$(ros2 pkg prefix ugv_gazebo)/share/ugv_gazebo/worlds/ugv_world.world \
  robot_sdf_file:=$(ros2 pkg prefix ugv_gazebo)/share/ugv_gazebo/models/ugv_rover/model.sdf \
  robot_spawn_x:=0.0 \
  robot_spawn_y:=0.0 \
  robot_spawn_z:=0.15
```

---

## 2. 리모콘 UI만 실행

터미널 2에서 시각화 리모콘만 실행한다. 이 명령은 Gazebo를 실행하지 않는다.

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_topic:=/map \
  map_display_mode:=auto \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  require_scan:=false \
  auto_mode_strategy:=mission_nav2 \
  publish_direct_cmd_vel:=false
```

UI에서 사용하는 주요 버튼:

- `SLAM MAPPING`: 기존 UI map/trace/path/goal overlay를 clear하고 LiDAR-only SLAM mapping backend를 시작한다.
- `STOP MAPPING`: mapping motion을 멈추고 저장/적용 대기 상태로 둔다.
- `SAVE MAP`: 현재 `/map`을 `~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml`로 저장한다.
- `APPLY FIXED MAP`: 저장된 map을 다시 `/map`으로 publish하여 UI fixed map으로 적용한다.
- `START PATROL`: fixed map / localization / mission backend 기준 순찰 시작 요청.
- `STOP`: goal cancel + STANDBY.
- `E-STOP`: EMERGENCY latch.
- `RESET`: E-STOP latch 해제 후 STANDBY.

키보드 조작:

- `W` / `↑`: 전진
- `S` / `↓`: 후진
- `A` / `←`: 좌회전
- `D` / `→`: 우회전
- `Space` 또는 `K`: 정지
- `E`: E-STOP
- `R`: RESET
- `P`: AUTO/PATROL 요청

주의: 키보드/WASD는 `/waver/manual_cmd_vel`만 publish한다. 실차 기본값에서 UI가 `/cmd_vel`을 직접 publish하면 안 된다.

---

## 3. SLAM Mapping 수동 workflow

터미널 1에서 Gazebo를 켜고, 터미널 2에서 리모콘 UI를 켠 뒤 다음 순서로 조작한다.

1. UI에서 `SLAM MAPPING` 클릭
2. WASD 또는 방향키로 천천히 주행하며 LiDAR SLAM map 생성
3. UI에서 `SAVE MAP` 클릭
4. UI에서 `APPLY FIXED MAP` 클릭
5. UI map 상태가 `MAP_FIXED_READY` 또는 fixed map 상태인지 확인
6. 필요 시 `START PATROL` 클릭

저장되는 map:

```text
~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml
~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.pgm
```

SLAM 중 UI가 `SLAM_LIVE`로 표시되어야 한다. `APPLY FIXED MAP` 후에는 map grid가 차체 yaw에 따라 같이 회전하지 않고, 로봇 pose/arrow/path만 map 위에서 움직여야 한다.

---

## 4. Gazebo + UI 통합 자동 검증

리모콘 UI까지 한 번에 띄워 자동으로 버튼/키보드 경로를 검증한다.

### 빠른 smoke test

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=true \
  use_operator_panel:=true \
  demo_script:=mapping_workflow_smoke \
  demo_close_on_finish:=true
```

### 넓은 공항맵 coverage test

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=true \
  use_operator_panel:=true \
  demo_script:=mapping_full_coverage \
  demo_close_on_finish:=true
```

`mapping_full_coverage`는 smoke보다 오래 주행해 공항맵의 더 넓은 부분을 SLAM으로 채운다. 논문용 “넓은 map coverage” 데이터는 이 명령 또는 수동 WASD coverage 후 저장한 map을 사용한다.

---

## 5. Headless 검증

GUI 부하 없이 Gazebo/SLAM/UI command path를 검증한다.

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
  use_gui:=false \
  use_operator_panel:=true \
  demo_script:=mapping_workflow_smoke \
  demo_close_on_finish:=true
```

최근 검증 기준:

- 3회 연속 성공
- `/scan` 평균 약 9.8 Hz
- map save/apply 성공
- depth/RGB-D SLAM topic 미사용
- `/cmd_vel` publisher: `safety_cmd_mux_node` 1개

결과 저장 경로:

```text
~/ros2_ws/FSD_Vehicle/experiments_result/latest/
```

실험 산출물은 GitHub에 올리지 않는다.

---

## 6. Mapping workflow checker

이 스크립트는 Gazebo/SLAM backend가 이미 떠 있는 상태에서 `/map` 변화, 저장, 적용 상태를 기록하는 보조 검사다. 단독으로 Gazebo나 gmapping을 띄우는 명령이 아니다.

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

python3 src/waver_patrol/scripts/run_mapping_mode_check.py \
  --output-root ~/ros2_ws/FSD_Vehicle/experiments_result \
  --experiment-name mapping_mode_manual \
  --duration-sec 60 \
  --save-after-sec 40 \
  --apply-after-save-sec 8
```

---

## 7. 주요 토픽 확인

```bash
ros2 topic list
ros2 node list
```

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
ros2 topic hz /plan
ros2 topic hz /local_plan
```

```bash
ros2 topic echo /waver/mapping_state
ros2 topic echo /waver/map_apply_state
ros2 topic echo /waver/map_saved_path
ros2 topic echo /waver/mission_state
ros2 topic echo /waver/safety_state
ros2 topic echo /waver/current_waypoint
ros2 topic echo /waver/active_nav_goal
```

안전 경로 확인:

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /waver/manual_cmd_vel
ros2 topic info -v /waver/cmd_vel_nav2
```

기대값:

- `/cmd_vel` publisher count = 1
- `/cmd_vel` publisher node = `safety_cmd_mux_node`
- UI는 `/waver/manual_cmd_vel`만 publish
- Nav2는 `/waver/cmd_vel_nav2`로 remap

---

## 8. Mid360 / PointCloud 확인

실차에서 높이 3m 이상 동적 객체를 판단하려면 z값이 있는 3D source가 필요하다. 2D LaserScan만으로 높이 3m 이상 객체라고 판단하면 안 된다.

```bash
ros2 topic hz /mid360_PointCloud2
ros2 topic echo /mid360_PointCloud2 --once
```

Object pipeline 확인:

```bash
ros2 topic echo /waver/lidar_objects
ros2 topic echo /waver/lidar_objects_map
ros2 topic echo /waver/elevated_dynamic_targets
ros2 topic echo /waver/moving_target_valid
ros2 topic echo /waver/height_filter_debug
ros2 topic echo /waver/ego_motion_compensation_debug
```

최종 target 조건:

```text
object_height_m >= 3.0
and z_valid == true
and dynamic_filter_pass == true
and ego_motion_compensated == true
```

3m는 이동 거리가 아니라 높이 기준이다.

---

## 9. 실차 적용 전 dry-run backend

실차를 움직이지 않고 topic, map, localization, mission, safety path만 확인한다.

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_sim_time:=false \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=false \
  include_existing_ugv_driver:=false \
  enable_test_publishers:=false \
  enable_deep_learning_stub:=false \
  enable_sound_stub:=false \
  enable_pointcloud_lidar_objects:=true \
  enable_moving_object_map_transform:=true \
  enable_moving_object_motion_filter:=true \
  default_mode:=STANDBY \
  remap_nav2_cmd_vel:=true \
  use_rviz:=false
```

리모콘 UI는 별도 터미널에서 실행한다.

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch ugv_tools waver_operator_panel.launch.py \
  profile:=real \
  require_scan:=true \
  map_display_mode:=auto \
  publish_direct_cmd_vel:=false \
  allow_subprocess_launches:=false \
  allow_mapping_launches:=true \
  allow_map_save_commands:=true \
  allow_localization_launches:=false
```

---

## 10. 실차 wheel-on 저속 backend

아래 명령은 wheel-off와 dry-run 점검을 통과한 뒤에만 사용한다.

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_sim_time:=false \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=true \
  include_existing_ugv_driver:=false \
  enable_test_publishers:=false \
  enable_deep_learning_stub:=false \
  enable_sound_stub:=false \
  enable_pointcloud_lidar_objects:=true \
  enable_moving_object_map_transform:=true \
  enable_moving_object_motion_filter:=true \
  default_mode:=STANDBY \
  remap_nav2_cmd_vel:=true \
  safety_max_linear_speed:=0.10 \
  safety_max_angular_speed:=0.30 \
  use_rviz:=false
```

실차 wheel-on 전 필수 확인:

```bash
ros2 topic info -v /cmd_vel
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /waver/safety_state
ros2 topic echo /waver/serial_bridge_state
```

금지 조건:

- `/cmd_vel` publisher가 2개 이상
- `ugv_driver`가 실차 profile에서 직접 serial/cmd_vel 경로를 소유
- `/scan` stale
- localization/TF 실패
- E-STOP active
- serial bridge 중복 실행

---

## 11. RViz 선택 실행

```bash
cd ~/ros2_ws/FSD_Vehicle
source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2 -d src/waver_patrol/rviz/pre_real_gazebo_validation.rviz
```

RViz Fixed Frame은 `map`으로 둔다.

---

## 12. 문제별 빠른 확인

### Gazebo에서 rover가 안 보일 때

```bash
ros2 node list | grep gazebo
ros2 topic echo /gazebo/model_states --once
ros2 pkg prefix ugv_gazebo
```

`gazebo_mapping_mode.launch.py` 기본값은 `ugv_world.world`와 `ugv_rover`를 사용한다.

### SLAM map이 점처럼 보일 때

```bash
ros2 topic hz /scan
ros2 topic echo /map --once
ros2 topic echo /waver/mapping_state
```

UI는 known free cell과 occupied cell을 같이 그린다. 그래도 map이 작으면 SLAM이 실패한 것이 아니라 coverage가 부족한 경우가 많다. `mapping_full_coverage` 또는 수동 WASD로 더 넓게 주행한 뒤 `SAVE MAP`을 누른다.

### 리모콘이 Gazebo를 같이 띄우는 것처럼 보일 때

UI-only 명령은 `ros2 launch ugv_tools waver_operator_panel.launch.py ...`이다. 이 명령은 Gazebo를 띄우지 않는다. 단, `SLAM MAPPING` 버튼은 설정된 `mapping_launch_command`에 따라 gmapping backend만 시작할 수 있다.
