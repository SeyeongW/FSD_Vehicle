from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    trial_launch = os.path.join(waver_share, "launch", "gazebo_moving_object_trial.launch.py")
    default_world = os.path.join(ugv_share, "worlds", "ugv_world.world")
    default_robot = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")
    obstacle_sdf = os.path.join(waver_share, "models", "dynamic_obstacle_box", "model.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("require_scan", default_value="false"),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robot_sdf_file", default_value=default_robot),
            DeclareLaunchArgument("obstacle_entity", default_value="dynamic_test_box"),
            DeclareLaunchArgument("obstacle_x_m", default_value="0.70"),
            DeclareLaunchArgument("obstacle_y_amplitude_m", default_value="0.65"),
            DeclareLaunchArgument("obstacle_period_sec", default_value="5.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(trial_launch),
                launch_arguments={
                    "use_gui": LaunchConfiguration("use_gui"),
                    "world_file": LaunchConfiguration("world_file"),
                    "robot_entity": "ugv_rover",
                    "robot_sdf_file": LaunchConfiguration("robot_sdf_file"),
                    "spawn_robot": "true",
                    "spawn_target": "false",
                    "enable_mission_stack": "true",
                    "enable_simple_nav2_cmd_sim": "true",
                    "enable_simple_nav2_avoidance": "true",
                    "enable_dynamic_obstacle_detour": "true",
                    "enable_gazebo_map_path_visualizer": "true",
                    "enable_moving_object_motion_filter": "false",
                    "enable_cluster_node": "false",
                    "enable_trial_logger": "false",
                    "record_bag": "false",
                    "require_scan": LaunchConfiguration("require_scan"),
                    "default_mode": "STANDBY",
                    "gazebo_sim_max_linear_speed": "0.25",
                }.items(),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                            "-entity",
                            LaunchConfiguration("obstacle_entity"),
                            "-file",
                            obstacle_sdf,
                            "-x",
                            LaunchConfiguration("obstacle_x_m"),
                            "-y",
                            "0.0",
                            "-z",
                            "0.25",
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    Node(
                        package="waver_patrol",
                        executable="gazebo_dynamic_obstacle_node",
                        name="gazebo_dynamic_obstacle_node",
                        output="screen",
                        parameters=[
                            {
                                "use_sim_time": True,
                                "entity_name": LaunchConfiguration("obstacle_entity"),
                                "x_m": ParameterValue(LaunchConfiguration("obstacle_x_m"), value_type=float),
                                "y_amplitude_m": ParameterValue(
                                    LaunchConfiguration("obstacle_y_amplitude_m"),
                                    value_type=float,
                                ),
                                "period_sec": ParameterValue(
                                    LaunchConfiguration("obstacle_period_sec"),
                                    value_type=float,
                                ),
                                "start_delay_sec": 0.0,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
