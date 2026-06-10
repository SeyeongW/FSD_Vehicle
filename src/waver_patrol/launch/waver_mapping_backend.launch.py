from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    default_source_map = os.path.join(ugv_share, "maps", "map.yaml")
    real_mapping_launch = os.path.join(waver_share, "launch", "waver_mapping_2d.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "backend",
                default_value="cartographer",
                description="cartographer, gmapping, or gazebo_live. gazebo_live is simulation-only.",
            ),
            DeclareLaunchArgument("source_map_yaml", default_value=default_source_map),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("start_workflow_manager", default_value="true"),
            DeclareLaunchArgument("start_lidar_bringup", default_value="false"),
            DeclareLaunchArgument("start_robot_pose_publisher", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("reveal_duration_sec", default_value="12.0"),
            DeclareLaunchArgument("save_dir", default_value="~/ros2_ws3/FSD_Vehicle/maps"),
            DeclareLaunchArgument("save_basename", default_value="waver_latest_map"),
            LogInfo(
                msg=(
                    "Waver mapping backend: no Gazebo process is started here. "
                    "Use backend:=gazebo_live for UI/save/apply workflow validation, "
                    "or backend:=cartographer/gmapping for LiDAR-only SLAM."
                )
            ),
            Node(
                package="waver_patrol",
                executable="mapping_workflow_manager_node",
                name="mapping_workflow_manager_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_workflow_manager")),
                parameters=[
                    {
                        "save_dir": LaunchConfiguration("save_dir"),
                        "save_basename": LaunchConfiguration("save_basename"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="gazebo_live_mapping_node",
                name="gazebo_live_mapping_node",
                output="screen",
                condition=LaunchConfigurationEquals("backend", "gazebo_live"),
                parameters=[
                    {
                        "map_yaml": LaunchConfiguration("source_map_yaml"),
                        "reveal_duration_sec": ParameterValue(
                            LaunchConfiguration("reveal_duration_sec"),
                            value_type=float,
                        ),
                        "save_dir": LaunchConfiguration("save_dir"),
                        "save_basename": LaunchConfiguration("save_basename"),
                        "auto_start": True,
                        "auto_save_on_complete": False,
                        "auto_apply_on_save": False,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_mapping_launch),
                launch_arguments={
                    "algorithm": "cartographer",
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "start_lidar_bringup": LaunchConfiguration("start_lidar_bringup"),
                    "start_robot_pose_publisher": LaunchConfiguration("start_robot_pose_publisher"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
                condition=LaunchConfigurationEquals("backend", "cartographer"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_mapping_launch),
                launch_arguments={
                    "algorithm": "gmapping",
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "start_lidar_bringup": LaunchConfiguration("start_lidar_bringup"),
                    "start_robot_pose_publisher": LaunchConfiguration("start_robot_pose_publisher"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                }.items(),
                condition=LaunchConfigurationEquals("backend", "gmapping"),
            ),
        ]
    )
