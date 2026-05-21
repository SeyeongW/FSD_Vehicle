from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("waver_patrol")
    default_config = os.path.join(share, "config", "moving_object_map_transform.yaml")

    config_file = LaunchConfiguration("config_file")
    input_type = LaunchConfiguration("input_type")
    input_topic = LaunchConfiguration("input_topic")
    target_frame = LaunchConfiguration("target_frame")
    fallback_frame = LaunchConfiguration("fallback_frame")
    lidar_frame = LaunchConfiguration("lidar_frame")
    output_topic = LaunchConfiguration("output_topic")
    marker_topic = LaunchConfiguration("marker_topic")
    enable_goal_relay = LaunchConfiguration("enable_goal_relay")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("input_type", default_value="pose_array"),
            DeclareLaunchArgument("input_topic", default_value="/waver/lidar_objects"),
            DeclareLaunchArgument("output_topic", default_value="/waver/lidar_objects_map"),
            DeclareLaunchArgument("marker_topic", default_value="/waver/moving_objects_map_marker"),
            DeclareLaunchArgument("lidar_frame", default_value=""),
            DeclareLaunchArgument("target_frame", default_value="map"),
            DeclareLaunchArgument("fallback_frame", default_value="odom"),
            DeclareLaunchArgument("enable_goal_relay", default_value="true"),
            LogInfo(
                msg=(
                    "Moving object map transform enabled. This publishes map/odom-frame object data only; "
                    "it does not publish /cmd_vel and does not override 4D radar mission commands."
                )
            ),
            Node(
                package="waver_patrol",
                executable="moving_object_map_transform_node",
                name="moving_object_map_transform_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_type": ParameterValue(input_type, value_type=str),
                        "input_topic": ParameterValue(input_topic, value_type=str),
                        "output_pose_array_topic": ParameterValue(output_topic, value_type=str),
                        "debug_marker_topic": ParameterValue(marker_topic, value_type=str),
                        "lidar_frame": ParameterValue(lidar_frame, value_type=str),
                        "target_frame": ParameterValue(target_frame, value_type=str),
                        "fallback_frame": ParameterValue(fallback_frame, value_type=str),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="moving_object_goal_relay_node",
                name="moving_object_goal_relay_node",
                output="screen",
                condition=IfCondition(enable_goal_relay),
                parameters=[
                    config_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "input_pose_array_topic": ParameterValue(output_topic, value_type=str),
                    },
                ],
            ),
        ]
    )
