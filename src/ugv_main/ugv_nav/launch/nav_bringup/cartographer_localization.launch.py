from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ugv_nav_share = get_package_share_directory("ugv_nav")
    default_pbstream = os.path.join(ugv_nav_share, "maps", "map.pbstream")
    default_config_dir = os.path.join(get_package_share_directory("cartographer"), "config")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_dir = LaunchConfiguration("cartographer_config_dir")
    config_basename = LaunchConfiguration("configuration_basename")
    pbstream_path = LaunchConfiguration("pbstream_path")
    resolution = LaunchConfiguration("resolution")
    publish_period_sec = LaunchConfiguration("publish_period_sec")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("cartographer_config_dir", default_value=default_config_dir),
            DeclareLaunchArgument("configuration_basename", default_value="localization_2d.lua"),
            DeclareLaunchArgument("pbstream_path", default_value=default_pbstream),
            DeclareLaunchArgument("resolution", default_value="0.05"),
            DeclareLaunchArgument("publish_period_sec", default_value="1.0"),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
                arguments=[
                    "-configuration_directory",
                    config_dir,
                    "-configuration_basename",
                    config_basename,
                    "-load_state_filename",
                    pbstream_path,
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("cartographer_ros"), "launch", "occupancy_grid.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "resolution": resolution,
                    "publish_period_sec": publish_period_sec,
                }.items(),
            ),
        ]
    )
