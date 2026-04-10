cmake_minimum_required(VERSION 3.8)

project(ugv_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)

install(DIRECTORY
  rviz
  launch
  urdf
  worlds
  models
  param
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/bird_manager.py
  scripts/ugv_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
