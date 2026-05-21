from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = LaunchConfiguration("config")
    include_existing_bringup = LaunchConfiguration("include_existing_bringup")
    start_serial_bridge = LaunchConfiguration("start_serial_bridge")

    ugv_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ugv_bringup"), "launch", "bringup_lidar.launch.py")
        ),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
        condition=IfCondition(include_existing_bringup),
    )

    serial_bridge = Node(
        package="waver_patrol",
        executable="serial_cmd_vel_bridge",
        name="waver_serial_cmd_vel_bridge",
        output="screen",
        parameters=[{"config": config}],
        condition=IfCondition(start_serial_bridge),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "include_existing_bringup",
                default_value="false",
                description="Enable only after checking ugv_driver serial ownership.",
            ),
            DeclareLaunchArgument("start_serial_bridge", default_value="true"),
            LogInfo(msg="Waver bringup keeps existing Waveshare bringup optional to avoid serial/cmd_vel conflicts."),
            ugv_bringup,
            serial_bridge,
        ]
    )
