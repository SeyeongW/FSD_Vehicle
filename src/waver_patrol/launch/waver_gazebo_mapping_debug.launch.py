from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    mapping_launch = os.path.join(waver_share, "launch", "gazebo_mapping_mode.launch.py")
    default_world = os.path.join(ugv_share, "worlds", "ugv_world.world")
    default_robot = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_operator_panel", default_value="true"),
            DeclareLaunchArgument("mapping_backend", default_value="gmapping"),
            DeclareLaunchArgument("scan_source_slam", default_value="gazebo_laser"),
            DeclareLaunchArgument("scan_source_safety", default_value="gazebo_laser"),
            DeclareLaunchArgument("demo_script", default_value=""),
            DeclareLaunchArgument("demo_close_on_finish", default_value="false"),
            DeclareLaunchArgument("save_dir", default_value="~/ros2_ws3/FSD_Vehicle/maps"),
            DeclareLaunchArgument("save_basename", default_value="waver_latest_map"),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robot_sdf_file", default_value=default_robot),
            LogInfo(
                msg=(
                    "Waver Gazebo mapping debug: Gazebo ugv_world + ugv_rover + LiDAR-only SLAM. "
                    "Bird/target pointcloud nodes are disabled to isolate /scan, /odom, TF, /map, and UI."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mapping_launch),
                launch_arguments={
                    "use_gui": LaunchConfiguration("use_gui"),
                    "use_operator_panel": LaunchConfiguration("use_operator_panel"),
                    "world_file": LaunchConfiguration("world_file"),
                    "robot_sdf_file": LaunchConfiguration("robot_sdf_file"),
                    "save_dir": LaunchConfiguration("save_dir"),
                    "save_basename": LaunchConfiguration("save_basename"),
                    "demo_script": LaunchConfiguration("demo_script"),
                    "demo_close_on_finish": LaunchConfiguration("demo_close_on_finish"),
                    "enable_gazebo_live_mapping": "false",
                    "mapping_launch_command": [
                        "ros2 launch waver_patrol waver_mapping_backend.launch.py ",
                        "backend:=",
                        LaunchConfiguration("mapping_backend"),
                        " use_sim_time:=true use_rviz:=false ",
                        "start_workflow_manager:=false start_lidar_bringup:=false ",
                        "start_robot_pose_publisher:=false scan_topic:=/scan_slam",
                    ],
                }.items(),
            ),
        ]
    )
