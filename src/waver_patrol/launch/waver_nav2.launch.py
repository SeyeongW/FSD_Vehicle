from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    existing_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ugv_nav"), "launch", "nav.launch.py")),
        launch_arguments={
            "use_localization": LaunchConfiguration("use_localization"),
            "use_localplan": LaunchConfiguration("use_localplan"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("include_existing_nav")),
    )
    serial_bridge = Node(
        package="waver_patrol",
        executable="serial_cmd_vel_bridge",
        name="waver_serial_cmd_vel_bridge",
        output="screen",
        parameters=[{"config": LaunchConfiguration("config")}],
        condition=IfCondition(LaunchConfiguration("start_serial_bridge")),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=""),
            DeclareLaunchArgument("include_existing_nav", default_value="true"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("use_localization", default_value="amcl"),
            DeclareLaunchArgument("use_localplan", default_value="dwa"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            LogInfo(msg="If ugv_nav launches ugv_driver, do not also start Waver serial bridge on the same command path."),
            existing_nav,
            serial_bridge,
        ]
    )
