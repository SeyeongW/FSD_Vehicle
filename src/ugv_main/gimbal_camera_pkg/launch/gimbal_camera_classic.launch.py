#!/usr/bin/env python3
"""Launch the PX4 CGO3 gimbal camera in Gazebo Classic (11) -- standalone.

Ported from PX4-ROS2's wang branch (camera/gimbal_camera/), adapted from a
copy-anywhere standalone folder to a normal colcon package: paths are resolved
via get_package_share_directory/get_package_prefix instead of a path relative
to this launch file.

Topics:
  ROS -> sim : /gimbal/yaw_cmd   (std_msgs/Float64)  target yaw   angle [rad]
               /gimbal/pitch_cmd (std_msgs/Float64)  target pitch angle [rad]
               /gimbal/roll_cmd  (std_msgs/Float64)  target roll  angle [rad]
  sim -> ROS : /gimbal/joint_states (sensor_msgs/JointState)
               /gimbal/image_raw    (sensor_msgs/Image)
               /gimbal/camera_info  (sensor_msgs/CameraInfo)
               /gimbal/imu          (sensor_msgs/Imu)

Drive it manually:
    ros2 run gimbal_camera_pkg gimbal_keyboard_control.py
Or track a bird visually:
    ros2 run gimbal_camera_pkg gimbal_bird_tracker.py --ros-args -p model_path:=yolov8n.pt
"""

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('gimbal_camera_pkg')
    world = os.path.join(pkg_share, 'worlds', 'gimbal_classic.world')
    models_dir = os.path.join(pkg_share, 'models')

    # libgimbal_joint_position_controller.so lands in lib/gimbal_camera_pkg
    # (this package's CMakeLists.txt install(TARGETS ... DESTINATION
    # lib/${PROJECT_NAME})), not share/ -- resolve it from the install prefix.
    plugins_dir = os.path.join(get_package_prefix('gimbal_camera_pkg'), 'lib', 'gimbal_camera_pkg')

    gui = LaunchConfiguration('gui')

    # model://gimbal/meshes/... and model://gimbal_classic resolve against
    # models_dir; the Classic model deliberately points its meshes at
    # model://gimbal/meshes/... (the gz-sim model's copy) rather than
    # duplicating 2.7 MB of STL, so both must stay under models/.
    set_model_path = AppendEnvironmentVariable('GAZEBO_MODEL_PATH', models_dir)
    set_plugin_path = AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', plugins_dir)

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world],
        output='screen',
    )
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'gui', default_value='true',
        description='Run gzclient too; set false for a headless render-only run'))
    ld.add_action(set_model_path)
    ld.add_action(set_plugin_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    return ld
