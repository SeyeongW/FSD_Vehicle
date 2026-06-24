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
    trial_launch = os.path.join(waver_share, "launch", "gazebo_moving_object_trial.launch.py")
    default_world = os.path.join(ugv_share, "worlds", "ugv_world.world")
    default_robot = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")
    default_map = os.path.join(ugv_share, "maps", "map.yaml")
    default_mapping_command = (
        "ros2 launch waver_patrol waver_mapping_backend.launch.py "
        "backend:=scan_mapper use_sim_time:=true use_rviz:=false "
        "start_workflow_manager:=false "
        "start_lidar_bringup:=false start_robot_pose_publisher:=false "
        "scan_topic:=/scan_slam"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_operator_panel", default_value="false"),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robot_sdf_file", default_value=default_robot),
            DeclareLaunchArgument("source_map_yaml", default_value=default_map),
            DeclareLaunchArgument("reveal_duration_sec", default_value="12.0"),
            DeclareLaunchArgument("save_dir", default_value="~/ugv_ws/FSD_Vehicle/maps"),
            DeclareLaunchArgument("save_basename", default_value="waver_latest_map"),
            DeclareLaunchArgument("demo_script", default_value=""),
            DeclareLaunchArgument("demo_close_on_finish", default_value="false"),
            DeclareLaunchArgument("mapping_launch_command", default_value=default_mapping_command),
            DeclareLaunchArgument("enable_gazebo_live_mapping", default_value="false"),
            DeclareLaunchArgument("robot_spawn_x", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_z", default_value="0.15"),
            LogInfo(
                msg=(
                    "Gazebo mapping mode: ugv_world.world + ugv_rover + live /map reveal. "
                    "Use /waver/mapping_command START_MAPPING and SAVE_MAP; fixed maps are separated on /map_fixed."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(trial_launch),
                launch_arguments={
                    "use_gui": LaunchConfiguration("use_gui"),
                    "world_file": LaunchConfiguration("world_file"),
                    "robot_entity": "ugv_rover",
                    "robot_sdf_file": LaunchConfiguration("robot_sdf_file"),
                    "robot_spawn_x": LaunchConfiguration("robot_spawn_x"),
                    "robot_spawn_y": LaunchConfiguration("robot_spawn_y"),
                    "robot_spawn_z": LaunchConfiguration("robot_spawn_z"),
                    "spawn_robot": "true",
                    "spawn_target": "false",
                    "enable_mission_stack": "true",
                    "enable_radar_command_bridge": "false",
                    "enable_target_goal_manager": "false",
                    "enable_pointcloud_lidar_objects": "false",
                    "enable_simple_nav2_cmd_sim": "false",
                    "enable_gazebo_map_path_visualizer": "true",
                    "gazebo_visualizer_publish_map": "false",
                    "enable_moving_object_motion_filter": "false",
                    "enable_cluster_node": "false",
                    "enable_trial_logger": "false",
                    "record_bag": "false",
                    "require_scan": "false",
                    "publish_static_map_to_odom_tf": "false",
                    "default_mode": "STANDBY",
                }.items(),
            ),
            Node(
                package="waver_patrol",
                executable="scan_republisher_node",
                name="scan_slam_republisher_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/scan",
                        "output_topic": "/scan_slam",
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="scan_republisher_node",
                name="scan_safety_republisher_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/scan",
                        "output_topic": "/scan_safety",
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="mapping_workflow_manager_node",
                name="mapping_workflow_manager_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "save_dir": LaunchConfiguration("save_dir"),
                        "save_basename": LaunchConfiguration("save_basename"),
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="mapping_path_publisher_node",
                name="mapping_path_publisher_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "map_frame": "map",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="gazebo_live_mapping_node",
                name="gazebo_live_mapping_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_gazebo_live_mapping")),
                parameters=[
                    {
                        "use_sim_time": True,
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
            Node(
                package="ugv_tools",
                executable="waver_remote_panel",
                name="waver_remote_panel",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_operator_panel")),
                parameters=[
                    {
                        "map_topic": "/map",
                        "fixed_map_topic": "/map_fixed",
                        "map_display_mode": "slam_live",
                        "global_path_topic": "/waver/mapping_path",
                        "local_path_topic": "/local_plan",
                        "lidar_required": False,
                        "publish_direct_cmd_vel": False,
                        "auto_mode_strategy": "mission_nav2",
                        "mapping_launch_command": LaunchConfiguration("mapping_launch_command"),
                        "allow_mapping_launches": True,
                        "demo_script": LaunchConfiguration("demo_script"),
                        "demo_close_on_finish": ParameterValue(
                            LaunchConfiguration("demo_close_on_finish"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
