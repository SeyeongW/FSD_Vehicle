from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    # 역할: 최종 실차 전 검증용 이름을 제공하되, 검증 로직은 이미 안정화한
    # gazebo_moving_object_trial.launch.py를 재사용한다.
    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    trial_launch = os.path.join(waver_share, "launch", "gazebo_moving_object_trial.launch.py")
    default_world = os.path.join(ugv_share, "worlds", "ugv_world.world")
    default_robot = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("trial_id", default_value="1"),
            DeclareLaunchArgument("scenario", default_value="elevated_dynamic_straight"),
            DeclareLaunchArgument("target_min_height_m", default_value="3.0"),
            DeclareLaunchArgument("min_dynamic_motion_m", default_value="0.2"),
            DeclareLaunchArgument("min_dynamic_velocity_mps", default_value="0.05"),
            DeclareLaunchArgument("target_z", default_value="3.2"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("enable_cluster_node", default_value="true"),
            DeclareLaunchArgument("enable_fake_camera_detection", default_value="true"),
            DeclareLaunchArgument("enable_fake_sound", default_value="true"),
            DeclareLaunchArgument("enable_experiment_logger", default_value="true"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("output_root", default_value="~/ros2_ws5/FSD_Vehicle/experiments_result"),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robot_sdf_file", default_value=default_robot),
            LogInfo(
                msg=[
                    "pre-real validation uses ugv_world.world + ugv_rover; scenario=",
                    LaunchConfiguration("scenario"),
                    ". H1/H2/H3 validate height>=3m dynamic target filtering, not 3m travel.",
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(trial_launch),
                launch_arguments={
                    "trial_id": LaunchConfiguration("trial_id"),
                    "target_min_height_m": LaunchConfiguration("target_min_height_m"),
                    "min_dynamic_motion_m": LaunchConfiguration("min_dynamic_motion_m"),
                    "min_dynamic_velocity_mps": LaunchConfiguration("min_dynamic_velocity_mps"),
                    "target_z": LaunchConfiguration("target_z"),
                    "use_gui": LaunchConfiguration("use_gui"),
                    "world_file": LaunchConfiguration("world_file"),
                    "robot_entity": "ugv_rover",
                    "robot_sdf_file": LaunchConfiguration("robot_sdf_file"),
                    "spawn_target": "false",
                    "spawn_ugv_bird_single": "false",
                    "spawn_ugv_bird_swarm": "false",
                    "enable_ugv_bird_manager": "false",
                    "enable_gazebo_bird_bridge": "false",
                    "enable_cluster_node": LaunchConfiguration("enable_cluster_node"),
                    "enable_fake_camera_classification": LaunchConfiguration("enable_fake_camera_detection"),
                    "enable_mission_stack": "true",
                    "enable_simple_nav2_cmd_sim": "true",
                    "enable_moving_object_motion_filter": "true",
                    "default_mode": "AUTO",
                    "enable_trial_logger": LaunchConfiguration("enable_experiment_logger"),
                    "record_bag": LaunchConfiguration("record_bag"),
                    "output_root": LaunchConfiguration("output_root"),
                    "require_scan": "false",
                    "gazebo_sim_max_linear_speed": "1.5",
                    "post_target_resume_cooldown_sec": "0.5",
                }.items(),
            ),
        ]
    )
