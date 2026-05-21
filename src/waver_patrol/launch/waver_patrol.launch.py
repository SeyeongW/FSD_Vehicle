from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("waver_patrol")
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "waver_bringup.launch.py")),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "include_existing_bringup": LaunchConfiguration("include_existing_bringup"),
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "waver_nav2.launch.py")),
        launch_arguments={
            "config": LaunchConfiguration("config"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_serial_bridge": "false",
        }.items(),
    )
    patrol = Node(
        package="waver_patrol",
        executable="patrol_run",
        name="waver_patrol_manager",
        output="screen",
        arguments=[LaunchConfiguration("waypoints"), "--live"],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=""),
            DeclareLaunchArgument("waypoints", default_value=os.path.join(pkg, "waypoints", "patrol_outdoor_demo.yaml")),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("include_existing_bringup", default_value="false"),
            bringup,
            nav2,
            patrol,
        ]
    )
