from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ugv_nav"), "launch", "nav.launch.py")),
        launch_arguments={
            "use_localization": LaunchConfiguration("use_localization"),
            "use_localplan": LaunchConfiguration("use_localplan"),
            "map": LaunchConfiguration("map"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("use_localization", default_value="amcl"),
            DeclareLaunchArgument("use_localplan", default_value="dwa"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            nav,
        ]
    )
