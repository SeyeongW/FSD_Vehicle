##gazebo 실행 ##

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch waver_patrol waver_nav2_radar_bird_mission.launch.py \
  use_nav2:=true \
  require_scan:=true \
  start_serial_bridge:=false \
  enable_test_publishers:=false \
  enable_pointcloud_lidar_objects:=true \
  enable_moving_object_map_transform:=true \
  default_mode:=STANDBY \
  remap_nav2_cmd_vel:=true \
  use_rviz:=false

##리모콘 실행 ##

ros2 launch ugv_tools waver_operator_panel.launch.py \
  map_topic:=/map \
  global_path_topic:=/plan \
  local_path_topic:=/local_plan \
  require_scan:=true \
  map_display_mode:=map_fixed \
  auto_mode_strategy:=mission_nav2 \
  publish_direct_cmd_vel:=false


##토픽 확인##

ros2 topic list
ros2 node list

ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /map
ros2 topic hz /amcl_pose

ros2 topic hz /mid360_PointCloud2
ros2 topic echo /mid360_PointCloud2 --once

ros2 topic echo /waver/mission_state
ros2 topic echo /waver/safety_state
ros2 topic echo /waver/current_waypoint
ros2 topic echo /waver/active_nav_goal

ros2 topic echo /waver/lidar_objects
ros2 topic echo /waver/lidar_objects_map
ros2 topic echo /waver/elevated_dynamic_targets
ros2 topic echo /waver/moving_target_valid

ros2 topic echo /plan
ros2 topic echo /local_plan
