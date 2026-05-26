from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    mapping_launch = os.path.join(waver_share, "launch", "gazebo_mapping_mode.launch.py")
    default_world = os.path.join(ugv_share, "worlds", "ugv_world.world")
    default_robot = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")
    default_output_dir = os.path.expanduser(
        "~/ros2_ws/FSD_Vehicle/experiments_result/paper_ready/bird_detection_validation"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_operator_panel", default_value="false"),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robot_sdf_file", default_value=default_robot),
            DeclareLaunchArgument("robot_spawn_x", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_z", default_value="0.15"),
            DeclareLaunchArgument("scenario_id", default_value="B3_DYNAMIC_BIRD_HIGH"),
            DeclareLaunchArgument("target_model_name", default_value="bird_test_target"),
            DeclareLaunchArgument("object_class", default_value="bird"),
            DeclareLaunchArgument("expected_bird", default_value="true"),
            DeclareLaunchArgument("expected_mission_trigger", default_value="true"),
            DeclareLaunchArgument("move_target_model", default_value="true"),
            DeclareLaunchArgument("target_min_height_m", default_value="3.0"),
            DeclareLaunchArgument("target_height_m", default_value="3.2"),
            DeclareLaunchArgument("min_dynamic_motion_m", default_value="0.2"),
            DeclareLaunchArgument("min_dynamic_velocity_mps", default_value="0.05"),
            DeclareLaunchArgument("output_dir", default_value=default_output_dir),
            DeclareLaunchArgument("enable_target_goal_manager", default_value="true"),
            LogInfo(
                msg=(
                    "Gazebo bird detection validation: ugv_world.world + ugv_rover + "
                    "synthetic Gazebo-model-state bird detector. This is a simulation "
                    "metric pipeline, not a real YOLO detector."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                launch_arguments={
                    "use_gui": LaunchConfiguration("use_gui"),
                    "use_operator_panel": LaunchConfiguration("use_operator_panel"),
                    "world_file": LaunchConfiguration("world_file"),
                    "robot_sdf_file": LaunchConfiguration("robot_sdf_file"),
                    "robot_spawn_x": LaunchConfiguration("robot_spawn_x"),
                    "robot_spawn_y": LaunchConfiguration("robot_spawn_y"),
                    "robot_spawn_z": LaunchConfiguration("robot_spawn_z"),
                    "demo_script": "",
                }.items(),
            ),
            Node(
                package="waver_patrol",
                executable="bird_detection_pipeline_node",
                name="bird_detection_pipeline_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "scenario_id": LaunchConfiguration("scenario_id"),
                        "target_model_name": LaunchConfiguration("target_model_name"),
                        "object_class": LaunchConfiguration("object_class"),
                        "expected_bird": ParameterValue(LaunchConfiguration("expected_bird"), value_type=bool),
                        "expected_mission_trigger": ParameterValue(
                            LaunchConfiguration("expected_mission_trigger"),
                            value_type=bool,
                        ),
                        "move_target_model": ParameterValue(LaunchConfiguration("move_target_model"), value_type=bool),
                        "initial_z": ParameterValue(LaunchConfiguration("target_height_m"), value_type=float),
                        "target_min_height_m": ParameterValue(
                            LaunchConfiguration("target_min_height_m"),
                            value_type=float,
                        ),
                        "min_dynamic_motion_m": ParameterValue(
                            LaunchConfiguration("min_dynamic_motion_m"),
                            value_type=float,
                        ),
                        "min_dynamic_velocity_mps": ParameterValue(
                            LaunchConfiguration("min_dynamic_velocity_mps"),
                            value_type=float,
                        ),
                        "detector_source": "gazebo_model_state_synthetic",
                        "detector_model": "gazebo_ground_truth_proxy",
                        "output_dir": LaunchConfiguration("output_dir"),
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="target_goal_manager_node",
                name="bird_target_goal_manager_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_target_goal_manager")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "require_motion": True,
                        "require_bird_confirmed": True,
                        "allow_radar_without_bird_confirmed": False,
                        "min_height_m": 3.0,
                        "min_depth_m": 0.5,
                        "max_target_distance_m": 30.0,
                        "goal_offset_distance_m": 1.5,
                        "subscribe_raw_lidar_objects": False,
                        "subscribe_lidar_objects_map": False,
                        "elevated_dynamic_targets_topic": "/waver/elevated_dynamic_targets",
                    }
                ],
            ),
        ]
    )
