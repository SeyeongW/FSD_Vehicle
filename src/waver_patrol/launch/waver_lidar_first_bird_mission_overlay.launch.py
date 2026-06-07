from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("waver_patrol")
    overlay_params = os.path.join(share, "config", "waver_lidar_first_bird_mission_overlay.yaml")

    include_base_launch = LaunchConfiguration("include_base_launch")
    base_launch_file = LaunchConfiguration("base_launch_file")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("include_base_launch", default_value="false"),
            DeclareLaunchArgument("base_launch_file", default_value="waver_real_bird_autonomy.launch.py"),
            DeclareLaunchArgument("params_file", default_value=overlay_params),
            DeclareLaunchArgument("enable_external_lidar_bridge", default_value="true"),
            DeclareLaunchArgument("enable_external_yolo_bridge", default_value="true"),
            DeclareLaunchArgument("enable_live_target_aim", default_value="true"),
            DeclareLaunchArgument("enable_body_tracking_observation", default_value="false"),
            DeclareLaunchArgument("enable_gimbal_feedback_bridge", default_value="true"),
            DeclareLaunchArgument("enable_target_departure_monitor", default_value="true"),
            DeclareLaunchArgument("enable_mission_reporter", default_value="true"),
            DeclareLaunchArgument("enable_experiment_logger", default_value="true"),
            DeclareLaunchArgument("enable_rosbag_recorder", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("waver_patrol"), "launch", base_launch_file])
                ),
                condition=IfCondition(include_base_launch),
            ),
            Node(
                package="waver_patrol",
                executable="external_lidar_dynamic_bridge_node",
                name="external_lidar_dynamic_bridge_node",
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration("enable_external_lidar_bridge")),
            ),
            Node(
                package="waver_patrol",
                executable="external_yolo_bridge_node",
                name="external_yolo_bridge_node",
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration("enable_external_yolo_bridge")),
            ),
            Node(
                package="waver_patrol",
                executable="live_target_aim_bridge_node",
                name="live_target_aim_bridge_node",
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration("enable_live_target_aim")),
            ),
            Node(
                package="waver_patrol",
                executable="target_observation_body_tracker_node",
                name="target_observation_body_tracker_node",
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration("enable_body_tracking_observation")),
            ),
            Node(
                package="waver_patrol",
                executable="waver_gimbal_feedback_bridge_node",
                name="waver_gimbal_feedback_bridge_node",
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration("enable_gimbal_feedback_bridge")),
            ),
            Node(
                package="waver_patrol",
                executable="target_departure_monitor_node",
                name="target_departure_monitor_node",
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration("enable_target_departure_monitor")),
            ),
            Node(
                package="waver_patrol",
                executable="mission_data_reporter_node",
                name="mission_data_reporter_node",
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration("enable_mission_reporter")),
            ),
            Node(
                package="waver_patrol",
                executable="experiment_data_logger_node",
                name="experiment_data_logger_node",
                parameters=[params_file],
                condition=IfCondition(LaunchConfiguration("enable_experiment_logger")),
            ),
        ]
    )
