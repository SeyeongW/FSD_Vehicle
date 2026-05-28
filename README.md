
---

## 프로젝트 핵심 평가 기준

이 프로젝트의 최종 목적은 SLAM map coverage 향상이 아니라 **조류탐지 자율주행 순찰**이다.

논문/보고서 판단 우선순위:

1. 조류탐지 precision, recall, F1, mAP, false positive/negative, latency, FPS
2. 조류 target 3D 위치화, z-valid, 높이/거리/방위각 오차, tracking continuity, ID switch
3. patrol mode에서만 유효한 bird mission trigger precision/recall/F1
4. 순찰/복귀/STOP/E-STOP 및 `/cmd_vel` single-publisher safety
5. LiDAR-only SLAM, map save/apply, fixed-map UI 반영

SLAM 지표는 조류탐지 자율주행을 가능하게 하는 support gate다. `map_known_ratio`가 좋아졌다는 사실만으로 조류탐지 성능 개선이라고 쓰면 안 된다.

---

## 0. 공통 준비

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths FSD_Vehicle/src --ignore-src -r -y
bash FSD_Vehicle/src/waver_patrol/scripts/waver_duplicate_package_check.sh
colcon build --packages-select \
  ugv_description ugv_bringup ugv_tools waver_patrol ugv_slam ugv_gazebo \
  --symlink-install
source install/setup.bash
export ROS_DOMAIN_ID=30
```

---

## 1. Gazebo Mapping Debug 실행

공항맵과 `ugv_rover`를 띄우고 SLAM mapping debug stack을 실행한다. 리모콘 UI를 별도 터미널에서 켤 때는 `use_operator_panel:=false`를 사용한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_gazebo_mapping_debug.launch.py \
  use_gui:=true \
  use_operator_panel:=false \
  mapping_backend:=gmapping \
  scan_source_slam:=gazebo_laser \
  scan_source_safety:=gazebo_laser \
  robot_spawn_x:=0.0 \
  robot_spawn_y:=0.0 \
  robot_spawn_z:=0.15
```


---

## 2. 리모콘 UI만 실행

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_topic:=/map \
  fixed_map_topic:=/map_fixed \
  map_display_mode:=auto \
  global_path_topic:=/waver/mapping_path \
  local_path_topic:=/local_plan \
  require_scan:=false \
  auto_mode_strategy:=mission_nav2 \
  publish_direct_cmd_vel:=false
```

UI에서 사용하는 주요 버튼:

- `SLAM MAPPING`: 기존 UI map/trace/path/goal overlay를 clear하고 LiDAR-only SLAM mapping backend를 시작한다.
- `STOP MAPPING`: mapping motion을 멈추고 저장/적용 대기 상태로 둔다.
- `SAVE MAP`: 현재 `/map`을 `~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml`로 저장한다.
- `APPLY FIXED MAP`: 저장된 map을 `/map_fixed`로 publish하고 UI source를 fixed map으로 전환한다.
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


---

## 3. SLAM Mapping 수동 workflow

터미널 1에서 Gazebo를 켜고, 터미널 2에서 리모콘 UI를 켠 뒤 다음 순서로 조작한다.

1. UI에서 `SLAM MAPPING` 클릭
2. WASD 또는 방향키로 천천히 주행하며 LiDAR SLAM map 생성
3. UI에서 `SAVE MAP` 클릭
4. UI에서 `APPLY FIXED MAP` 클릭
5. UI map 상태가 `map source: FIXED_MAP_READY` 및 `MAP_FIXED_READY`인지 확인
6. 필요 시 `START PATROL` 클릭

저장되는 map:

```text
~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml
~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.pgm
```

SLAM 중 UI가 `SLAM_LIVE`로 표시되어야 한다. mapping 중 `/map` publisher는 SLAM live source 하나여야 하며, fixed map은 `/map_fixed`로 분리된다. `APPLY FIXED MAP` 후에는 map grid가 차체 yaw에 따라 같이 회전하지 않고, 로봇 pose/arrow/path만 map 위에서 움직여야 한다.

---

생성 주요 파일:

```text
experiments_result/paper_ready/bird_detection_10runs/final_10runs/summary_10runs.csv
experiments_result/paper_ready/bird_detection_10runs/statistics_10runs.csv
experiments_result/paper_ready/bird_detection_10runs/final_judgement.md
```

검증 topic:

```bash
ros2 topic echo /bird/detections_2d
ros2 topic echo /bird/detections_3d
ros2 topic echo /bird/tracks
ros2 topic echo /bird/metrics
ros2 topic echo /bird/mission_target
ros2 topic info -v /cmd_vel
```


---

검증 기준:

- 최신 코드 수정 후에는 같은 commit에서 Gazebo + 리모콘 UI 반복 검증을 다시 수집한다.
- `/scan` 평균 약 13.2 Hz
- map save/apply 성공
- depth/RGB-D SLAM topic 미사용
- `/cmd_vel` publisher: `safety_cmd_mux_node` 1개
- 2026-05-26 SLAM 보정 후 full coverage 1회 검증:
  - `/scan` 평균 약 17.4 Hz
  - `map_known_ratio`: 약 0.332
  - `known_cell_count`: 183633
  - dots-only map 아님, runway/apron/service-road 경계가 연속 선으로 저장됨

공항 월드의 runway 표식 상당수는 visual-only decal이라 2D LiDAR가 볼 수 없다.
`ugv_world.world`에는 SLAM 검증용 저상 LiDAR-visible curb collision을 추가해
실제 `/scan` 기반 gmapping이 선형 구조를 누적할 수 있게 했다.

결과 저장 경로:

```text
~/ros2_ws/FSD_Vehicle/experiments_result/paper_ready/ai_gazebo_ui_10runs/
```

---

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
ros2 topic echo /waver/current_map_source
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
ros2 topic info -v /waver/mode
ros2 topic info -v /map
ros2 topic info -v /map_fixed
ros2 topic info -v /waver/manual_cmd_vel
ros2 topic info -v /waver/cmd_vel_nav2
```

기대값:

- `/cmd_vel` publisher count = 1
- `/cmd_vel` publisher node = `safety_cmd_mux_node`
- `/waver/mode` publisher count = 1
- `/waver/mode` publisher node = `mission_patrol_manager_node`
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


---

## 9. 실차 적용 전 dry-run backend

실차를 움직이지 않고 topic, map, localization, mission, safety path만 확인한다.

```bash
cd ~/ros2_ws
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
cd ~/ros2_ws
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

## 10. 실차 조류탐지 real backend

실차용 기본 launch는 fake/test/Gazebo bird publisher를 실행하지 않는다. 카메라 모델이 없으면 `bird_confirmed=false`가 유지되어 bird approach mission은 막힌다.

### Dry-run / no serial

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  start_serial_bridge:=false \
  enable_waver_base_driver:=false \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  scan_source_safety:=mid360 \
  scan_source_slam:=mid360 \
  odom_source:=ekf \
  map:=$HOME/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml \
  bird_model_path:=$HOME/models/bird_yolov8n.pt
```

### Wheel-off serial test

바퀴를 띄우고 물리 E-STOP을 잡은 상태에서만 실행한다.

```bash
ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  enable_waver_base_driver:=true \
  start_serial_bridge:=false \
  serial_port:=/dev/serial/by-id/<WAVER_SERIAL_ID> \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  scan_source_safety:=mid360 \
  scan_source_slam:=mid360 \
  odom_source:=ekf \
  map:=$HOME/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml \
  bird_model_path:=$HOME/models/bird_yolov8n.pt
```

### Wheel-on 저속 backend

아래 명령은 wheel-off와 dry-run 점검을 통과한 뒤에만 사용한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_real_bird_autonomy.launch.py \
  enable_waver_base_driver:=true \
  start_serial_bridge:=false \
  serial_port:=/dev/serial/by-id/<WAVER_SERIAL_ID> \
  default_mode:=STANDBY \
  safety_max_linear_speed:=0.05 \
  safety_max_angular_speed:=0.20 \
  scan_source_safety:=mid360 \
  scan_source_slam:=mid360 \
  odom_source:=ekf \
  map:=$HOME/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml \
  bird_model_path:=$HOME/models/bird_yolov8n.pt
```

실차 wheel-on 전 필수 확인:

```bash
ros2 topic info -v /cmd_vel
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic echo /waver/safety_state
ros2 topic echo /waver/base_driver_state
bash FSD_Vehicle/src/waver_patrol/scripts/waver_duplicate_package_check.sh
bash FSD_Vehicle/src/waver_patrol/scripts/waver_real_preflight_check.sh --strict
bash FSD_Vehicle/src/waver_patrol/scripts/waver_cmd_chain_check.sh
bash FSD_Vehicle/src/waver_patrol/scripts/waver_bird_autonomy_health_check.sh
```



