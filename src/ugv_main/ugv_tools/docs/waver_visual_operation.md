# Waver Visual Operation Guide

## 권장 구조

Waver 실차 운용은 두 역할로 나누는 것을 기본으로 한다.

1. Jetson robot backend
   - 센서, serial bridge, 안전 정지, 하드웨어 연결을 담당한다.
   - GUI를 띄우지 않는 headless 운용을 기본으로 한다.
   - 실행 파일: `waver_robot_backend.launch.py`

2. Operator PC panel
   - 리모콘, RViz, Foxglove, 로그 확인, 지도 확인을 담당한다.
   - 같은 ROS 2 네트워크에 붙어서 `/cmd_vel`, `/odom`, `/scan`, `/waver/*` 토픽을 본다.
   - 방향키/버튼은 `/waver/manual_cmd_vel`만 발행하고, 최종 `/cmd_vel`은 Jetson의 `safety_cmd_mux_node`가 낸다.
   - 실행 파일: `waver_operator_panel.launch.py`

Nav2/mission 통합 운용에서는 Jetson backend를 `waver_patrol`에서 실행하고, 조작 PC는 `ugv_tools` 리모콘만 실행한다.
Jetson에 모니터/키보드를 직접 꽂아 legacy 단독 테스트를 할 때만 `waver_real_control.launch.py`를 사용한다.

## 왜 이렇게 나누는가

- Jetson은 배터리, 온도, serial 포트, 센서 처리에 집중해야 한다.
- RViz/Gazebo/Foxglove 같은 시각화는 GPU/화면/네트워크 부하가 있어 조작 PC에서 띄우는 구성이 흔하다.
- 조작 PC가 꺼져도 Jetson backend는 `/cmd_vel` timeout과 serial stop burst로 정지해야 한다.
- 실차 serial 포트는 한 프로세스만 잡아야 하므로, operator PC에서는 serial bridge를 실행하지 않는다.

## ROS 2 네트워크 기본값

Jetson과 조작 PC 둘 다 같은 값을 사용한다.

```bash
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0
```

같은 Wi-Fi/LAN에 있고 multicast가 막혀 있지 않으면 ROS 2 DDS discovery로 서로를 찾는다.
회사/학교망처럼 multicast가 막힌 환경에서는 Fast DDS Discovery Server나 Foxglove bridge/WebSocket 구성을 사용한다.

## Jetson에서 실행: Nav2/mission 통합 권장

이 구성이 현재 Waver 조류탐지 순찰 실증용 기본 구조다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0

ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=false \
  enable_test_publishers:=false \
  default_mode:=STANDBY \
  remap_nav2_cmd_vel:=true
```

실차에 serial bridge까지 붙일 때는 기존 `ugv_driver`나 `app.py`가 같은 serial을 잡고 있지 않은지 확인한 뒤에만 켠다.

```bash
ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=true \
  include_existing_ugv_driver:=false \
  enable_test_publishers:=false \
  default_mode:=STANDBY \
  remap_nav2_cmd_vel:=true
```

중요 확인:

```bash
ros2 topic info -v /cmd_vel
```

publisher는 `safety_cmd_mux_node` 하나여야 한다. Nav2 출력은 `/waver/cmd_vel_nav2`로 remap되어야 한다.
이 상태에서는 로봇이 정지 대기하며, 리모콘의 AUTO 버튼을 누르면 `/waver/mode=AUTO`가 발행되어 순찰 goal/path 생성이 시작된다.

## Jetson에서 실행: legacy ugv_tools backend

아래 방식은 기존 ugv_tools 기반 하드웨어/수동 확인용이다. Nav2 mission 통합 실증에는 위의 `waver_patrol` backend를 사용한다.
Jetson 하드웨어 경로는 둘 중 하나만 고른다.

1. `권장 A`: 원본 FSD/Waveshare bringup을 사용한다.
   - `ugv_bringup/bringup_lidar.launch.py`가 `ugv_driver`, LiDAR, odom/base node를 함께 띄운다.
   - 이때 Waver serial bridge는 켜지 않는다.
   - 로컬 PC 리모콘은 `/cmd_vel`만 발행하고, Jetson의 기존 `ugv_driver`가 serial로 보낸다.

2. `대안 B`: Waver serial bridge를 직접 사용한다.
   - 기존 `ugv_driver`나 Waveshare `app.py`를 반드시 꺼야 한다.
   - 센서 bringup은 별도로 확인해야 한다.

### 권장 A: 기존 ugv_bringup 사용

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0

ros2 launch ugv_tools waver_robot_backend.launch.py \
  use_existing_ugv_bringup:=true \
  existing_ugv_serial_port:=/dev/ttyTHS1 \
  start_serial_bridge:=false \
  control_config_file:=/home/chotaehyun/ros2_ws/install/ugv_tools/share/ugv_tools/config/waver_4wd_control.yaml \
  require_scan:=true \
  min_valid_scan_points:=40
```

`/dev/ttyTHS1`은 이 레포의 기존 `ugv_driver` 기본값이다. 실제 Waver가 `/dev/ttyTHS0`이면 `existing_ugv_serial_port:=/dev/ttyTHS0`로 바꾼다.

### 대안 B: Waver serial bridge 직접 사용

기존 `ugv_driver` 또는 Waveshare `app.py`가 같은 serial을 사용 중이면 먼저 중지한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0

ros2 launch ugv_tools waver_robot_backend.launch.py \
  use_existing_ugv_bringup:=false \
  start_serial_bridge:=true \
  serial_port:=/dev/ttyTHS0 \
  baudrate:=115200 \
  require_scan:=true \
  min_valid_scan_points:=40
```

두 모드를 동시에 켜지 않는다.

## 조작 PC에서 실행

조작 PC에도 같은 workspace가 있거나, 최소한 `ugv_tools`가 빌드되어 있어야 한다.
AUTO 버튼은 별도 자율주행 프로세스를 새로 띄우지 않고 `/waver/mode=AUTO`를 발행한다.
따라서 Jetson에서 `waver_patrol waver_nav2_radar_bird_mission.launch.py`가 먼저 실행 중이어야 한다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=27
export ROS_LOCALHOST_ONLY=0

ros2 launch ugv_tools waver_operator_panel.launch.py \
  control_config_file:=/home/chotaehyun/ros2_ws/install/ugv_tools/share/ugv_tools/config/waver_4wd_control.yaml \
  waypoint_file:=/home/chotaehyun/ros2_ws/install/ugv_tools/share/ugv_tools/waypoints/waver_3m_patrol.yaml \
  require_scan:=true \
  min_valid_scan_points:=40 \
  max_patrol_radius_m:=3.0 \
  auto_mode_strategy:=mission_nav2
```

AUTO 중 방향키/WASD를 누르면 `/waver/mode`는 AUTO로 유지되고 `/waver/manual_cmd_vel`만 잠깐 발행된다.
키를 떼면 manual 후보가 0이 되어 `safety_cmd_mux_node`가 다시 `/waver/cmd_vel_nav2`를 통과시킨다.

## 한 대에서 테스트

Jetson에 모니터/키보드가 붙어 있고, legacy 수동/단순 순찰 경로를 한 번에 확인할 때만 사용한다.
Nav2 mission 통합 실증에서는 이 launch 대신 `waver_patrol` backend와 `waver_operator_panel.launch.py`를 사용한다.

```bash
ros2 launch ugv_tools waver_real_control.launch.py \
  start_serial_bridge:=true \
  serial_port:=/dev/ttyTHS0 \
  control_config_file:=/home/chotaehyun/ros2_ws/install/ugv_tools/share/ugv_tools/config/waver_4wd_control.yaml \
  waypoint_file:=/home/chotaehyun/ros2_ws/install/ugv_tools/share/ugv_tools/waypoints/waver_3m_patrol.yaml \
  require_scan:=true \
  min_valid_scan_points:=40 \
  max_patrol_radius_m:=3.0
```

## 속도와 4륜 Waver 구동

Waver 실차 제어 기본 상한은 `max_linear_speed=0.30 m/s`로 둔다.
리모콘 시작 속도는 `0.18 m/s`이며, GUI slider에서 `0.30 m/s`까지 올릴 수 있다.

속도/가속도/scan 안전값은 아래 파일에서 수정한다.

```text
src/FSD_Vehicle/src/ugv_main/ugv_tools/config/waver_4wd_control.yaml
```

주로 바꾸는 값:

- `waver_remote_panel.default_speed`: GUI 리모콘 시작 속도
- `waver_remote_panel.max_linear_speed`: GUI 수동 조작 최대 선속도
- `waver_gazebo_patrol.max_linear_speed`: AUTO waypoint 순찰 최대 선속도
- `waver_cmd_vel_serial_bridge.max_demo_ratio`: WAVE ROVER JSON `L/R` PWM 비율 상한
- `hard_stop_distance_m`, `slow_down_distance_m`: LiDAR 안전거리

WAVE ROVER 하위 제어기는 `{"T":1,"L":left,"R":right}` 형식의 좌/우 명령을 받는다.
4륜 Waver에서는 `L`이 좌측 앞/뒤 바퀴 쌍, `R`이 우측 앞/뒤 바퀴 쌍이다.
엔코더 폐루프가 없는 구성에서는 `0.30 m/s`가 실제 보장 속도가 아니라 ROS 상위 명령 상한이며, 노면과 배터리 상태에 따라 실제 속도는 반드시 실측 보정해야 한다.

## Gazebo 리모콘 테스트

LiDAR safety까지 안정적으로 확인하려면 최소 모델을 사용한다.

```bash
ros2 launch ugv_tools waver_gazebo_test.launch.py \
  use_minimal_model:=true \
  gui:=true \
  start_remote_panel:=true \
  control_config_file:=/home/chotaehyun/ros2_ws/install/ugv_tools/share/ugv_tools/config/waver_4wd_control.yaml \
  waypoint_file:=/home/chotaehyun/ros2_ws/install/ugv_tools/share/ugv_tools/waypoints/waver_3m_patrol.yaml \
  require_scan:=true \
  min_valid_scan_points:=0 \
  max_patrol_radius_m:=3.0
```

원본 `ugv_rover` 모델은 `libros2_livox.so`가 없으면 LiDAR plugin이 뜨지 않을 수 있다.
그 경우 구동 경로만 확인할 때 `require_scan:=false`를 사용하고, 실차 주행에는 사용하지 않는다.

## 순찰 반경 제한

`waver_gazebo_patrol`은 기본적으로 `patrol_center_x=0.0`, `patrol_center_y=0.0`, `max_patrol_radius_m=3.0`을 사용한다.

웨이포인트는 아래 파일에서 수정한다.

```text
src/FSD_Vehicle/src/ugv_main/ugv_tools/waypoints/waver_3m_patrol.yaml
```

- CSV waypoint가 반경 밖이면 3m 원 경계로 자동 clamp한다.
- RViz clicked goal도 같은 제한을 받는다.
- `clamp_waypoints_to_radius:=false`로 바꾸면 반경 밖 waypoint를 버린다.

실외 첫 실증에서는 기준점에서 3m 이내, 저속, 사람 없는 통제 공간, 수동 E-Stop 담당자 배치를 전제로 한다.

## 리모콘에서 보이는 값

- `CONTROL`: `STANDBY`, `MANUAL`, `AUTO`, `EMERGENCY` 중 현재 최종 제어 모드
- `E-STOP`: 긴급정지 latch 상태
- `SCAN`: LiDAR 기반 `clear`, `caution`, `slow`, `stop`, `sensor_stale`, `sensor_degraded`
- `FINAL CMD`: 실제 `/cmd_vel` 선속도와 각속도
- `ODOM`: `/odom` 기준 현재 위치
- `PATROL`: waypoint 순찰 상태와 이유
- `AUTO CANDIDATE`: AUTO 노드가 제안한 후보 명령. 최종 `/cmd_vel`은 리모콘 safety filter 통과 후 발행된다.

## 실차 전 확인

1. 바퀴를 공중에 띄우고 `start_serial_bridge:=true`로 방향과 stop burst를 확인한다.
2. 기존 bringup 모드라면 `use_existing_ugv_bringup:=true`, `start_serial_bridge:=false` 조합인지 확인한다.
3. Waver serial bridge 모드라면 `ugv_driver`, Waveshare `app.py`, 다른 serial 제어 프로세스가 `/dev/ttyTHS0`를 잡지 않는지 확인한다.
3. `/scan`, `/odom` 수신을 확인한다.
4. 리모콘 `E-STOP`, `RESET`, `STOP`, 방향키 release 정지를 확인한다.
5. 통제된 공간에서 `max_patrol_radius_m:=3.0` 그대로 AUTO를 누른다.
