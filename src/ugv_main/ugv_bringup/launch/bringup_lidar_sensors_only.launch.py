from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_lidar = os.path.join(
        get_package_share_directory("ugv_bringup"),
        "launch",
        "bringup_lidar.launch.py",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("pub_odom_tf", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value="bringup"),
            DeclareLaunchArgument("start_base_feedback", default_value="false"),
            DeclareLaunchArgument("feedback_serial_port", default_value=""),
            DeclareLaunchArgument("feedback_baudrate", default_value="115200"),
            DeclareLaunchArgument("base_node_executable", default_value="base_node"),
            DeclareLaunchArgument("enable_ldlidar", default_value="false"),
            DeclareLaunchArgument("enable_rf2o", default_value="false"),
            LogInfo(
                msg=(
                    "Starting UGV sensor-only bringup: robot state, lidar and odom only. "
                    "Legacy ugv_driver is disabled so /cmd_vel must pass through safety mux."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(bringup_lidar),
                launch_arguments={
                    "pub_odom_tf": LaunchConfiguration("pub_odom_tf"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "rviz_config": LaunchConfiguration("rviz_config"),
                    "start_driver": "false",
                    "legacy_driver_enabled": "false",
                    "start_base_feedback": LaunchConfiguration("start_base_feedback"),
                    "feedback_serial_port": LaunchConfiguration("feedback_serial_port"),
                    "feedback_baudrate": LaunchConfiguration("feedback_baudrate"),
                    "base_node_executable": LaunchConfiguration("base_node_executable"),
                    "enable_ldlidar": LaunchConfiguration("enable_ldlidar"),
                    "enable_rf2o": LaunchConfiguration("enable_rf2o"),
                }.items(),
            ),
        ]
    )
