##gazebo 실행
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_gazebo_rover_only.launch.py \
  use_gui:=true \
  robot_spawn_x:=0.0 \
  robot_spawn_y:=0.0 \
  robot_spawn_z:=0.15 \
  robot_spawn_yaw:=0.0

##리모콘 실행
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_topic:=/map \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  require_scan:=false \
  auto_mode_strategy:=mission_nav2 \
  map_display_mode:=map_fixed \
  publish_direct_cmd_vel:=false
