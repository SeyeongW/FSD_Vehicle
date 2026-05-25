from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    # 역할: CI/반복 실험용 headless wrapper다.
    waver_share = get_package_share_directory("waver_patrol")
    pre_real_launch = os.path.join(waver_share, "launch", "gazebo_pre_real_validation.launch.py")
    return LaunchDescription(
        [
            DeclareLaunchArgument("trial_id", default_value="1"),
            DeclareLaunchArgument("scenario", default_value="elevated_dynamic_straight"),
            DeclareLaunchArgument("target_min_height_m", default_value="3.0"),
            DeclareLaunchArgument("min_dynamic_motion_m", default_value="0.2"),
            DeclareLaunchArgument("min_dynamic_velocity_mps", default_value="0.05"),
            DeclareLaunchArgument("target_z", default_value="3.2"),
            DeclareLaunchArgument("enable_cluster_node", default_value="false"),
            DeclareLaunchArgument("enable_trial_logger", default_value="false"),
            DeclareLaunchArgument("enable_fake_camera_detection", default_value="true"),
            DeclareLaunchArgument("enable_fake_sound", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("output_root", default_value="~/ros2_ws/experiments_result"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(pre_real_launch),
                launch_arguments={
                    "trial_id": LaunchConfiguration("trial_id"),
                    "scenario": LaunchConfiguration("scenario"),
                    "target_min_height_m": LaunchConfiguration("target_min_height_m"),
                    "min_dynamic_motion_m": LaunchConfiguration("min_dynamic_motion_m"),
                    "min_dynamic_velocity_mps": LaunchConfiguration("min_dynamic_velocity_mps"),
                    "target_z": LaunchConfiguration("target_z"),
                    "enable_cluster_node": LaunchConfiguration("enable_cluster_node"),
                    "enable_experiment_logger": LaunchConfiguration("enable_trial_logger"),
                    "enable_fake_camera_detection": LaunchConfiguration("enable_fake_camera_detection"),
                    "enable_fake_sound": LaunchConfiguration("enable_fake_sound"),
                    "use_gui": "false",
                    "record_bag": LaunchConfiguration("record_bag"),
                    "output_root": LaunchConfiguration("output_root"),
                }.items(),
            ),
        ]
    )
