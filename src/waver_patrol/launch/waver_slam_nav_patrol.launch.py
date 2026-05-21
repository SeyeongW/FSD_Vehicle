from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("waver_patrol")
    slam_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ugv_nav"), "launch", "slam_nav.launch.py")),
        launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
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
            DeclareLaunchArgument("waypoints", default_value=os.path.join(pkg, "waypoints", "patrol_outdoor_demo.yaml")),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            LogInfo(msg="SLAM plus patrol is for low-speed supervised tests only; require good localization before motion."),
            slam_nav,
            patrol,
        ]
    )
