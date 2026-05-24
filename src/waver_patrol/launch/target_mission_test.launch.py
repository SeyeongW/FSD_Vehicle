from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    # 역할: Gazebo 없이 mission stack과 fake target/camera publisher만 검증할 때 사용한다.
    waver_share = get_package_share_directory("waver_patrol")
    mission_launch = os.path.join(waver_share, "launch", "waver_nav2_radar_bird_mission.launch.py")
    return LaunchDescription(
        [
            DeclareLaunchArgument("require_scan", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_launch),
                launch_arguments={
                    "use_nav2": "false",
                    "use_sim_odom": "true",
                    "require_scan": LaunchConfiguration("require_scan"),
                    "start_serial_bridge": "false",
                    "enable_test_publishers": "true",
                    "enable_experiment_logger": "false",
                    "default_mode": "AUTO",
                    "enable_moving_object_map_transform": "true",
                }.items(),
            ),
        ]
    )
