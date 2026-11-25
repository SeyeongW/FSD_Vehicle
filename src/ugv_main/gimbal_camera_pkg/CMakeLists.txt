cmake_minimum_required(VERSION 3.8)
project(gimbal_camera_pkg)

find_package(ament_cmake REQUIRED)

# Gazebo Classic 11 pulls in ignition-cmake2 -> FindJSONCPP -> the system
# jsoncppConfig.cmake, which still declares cmake_minimum_required(VERSION 3.0).
# CMake 4 removed compatibility with < 3.5, so find_package(gazebo) dies inside
# a file we do not own. Keep treating those as 3.5.
set(CMAKE_POLICY_VERSION_MINIMUM 3.5 CACHE STRING "" FORCE)

# Gazebo Classic is OPTIONAL: this package also builds on machines without a
# simulator (e.g. the Jetson), where the gimbal joint controller plugin is
# simply never loaded.
find_package(gazebo QUIET)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(gazebo_FOUND)
  add_library(gimbal_joint_position_controller SHARED
    plugins/gimbal_joint_position_controller.cpp)
  target_include_directories(gimbal_joint_position_controller
    PRIVATE ${GAZEBO_INCLUDE_DIRS})
  target_link_directories(gimbal_joint_position_controller
    PRIVATE ${GAZEBO_LIBRARY_DIRS})
  target_link_libraries(gimbal_joint_position_controller
    ${GAZEBO_LIBRARIES}
    rclcpp::rclcpp
    ${std_msgs_TARGETS})
  install(TARGETS gimbal_joint_position_controller
    LIBRARY DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS
    "gimbal_camera_pkg: Gazebo Classic not found — skipping the gimbal "
    "joint controller plugin build (expected on a machine with no simulator).")
endif()

install(DIRECTORY
  worlds
  models
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/gimbal_keyboard_control.py
  scripts/gimbal_bird_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
