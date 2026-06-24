from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Airport Gazebo + mission backend + operator map UI demo.

    This launch is intentionally a wrapper around existing packages:
      - ugv_gazebo provides ugv_world.world and ugv_rover.
      - waver_patrol provides the Gazebo mission/safety/perception backend.
      - ugv_tools provides the operator panel.

    The rover starts in STANDBY. It only patrols when the operator panel sends
    START_PATROL/AUTO_MODE or when the same command is published manually.
    """

    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    ugv_tools_share = get_package_share_directory("ugv_tools")

    backend_launch = os.path.join(waver_share, "launch", "gazebo_dynamic_obstacle_avoidance.launch.py")
    panel_launch = os.path.join(ugv_tools_share, "launch", "waver_operator_panel.launch.py")
    default_world = os.path.join(ugv_share, "worlds", "ugv_world.world")
    default_robot = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")
    default_waypoints = os.path.join(waver_share, "waypoints", "gazebo_airport_patrol_large.yaml")
    default_map = os.path.join(ugv_share, "maps", "map.yaml")

    source_setup = (
        "source /opt/ros/humble/setup.bash && "
        "source $HOME/ros2_ws5/FSD_Vehicle/install/setup.bash"
    )
    default_mapping_command = (
        f"bash -lc '{source_setup} && "
        "ros2 launch slam_gmapping mapping.launch.py use_sim_time:=true'"
    )
    default_map_save_command = (
        f"bash -lc '{source_setup} && "
        "mkdir -p $HOME/ros2_ws5/FSD_Vehicle/maps && "
        "ros2 run nav2_map_server map_saver_cli -f $HOME/ros2_ws5/FSD_Vehicle/maps/patrol_map'"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_operator_panel", default_value="true"),
            DeclareLaunchArgument("require_scan", default_value="false"),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robot_sdf_file", default_value=default_robot),
            DeclareLaunchArgument("map_yaml", default_value=default_map),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            DeclareLaunchArgument("target_z", default_value="3.2"),
            DeclareLaunchArgument("target_min_height_m", default_value="3.0"),
            DeclareLaunchArgument("min_dynamic_motion_m", default_value="0.2"),
            DeclareLaunchArgument("min_dynamic_velocity_mps", default_value="0.05"),
            DeclareLaunchArgument("spawn_ground_dynamic_obstacle", default_value="false"),
            DeclareLaunchArgument("gazebo_goal_tolerance_m", default_value="0.85"),
            DeclareLaunchArgument("mapping_launch_command", default_value=default_mapping_command),
            DeclareLaunchArgument("map_save_command", default_value=default_map_save_command),
            DeclareLaunchArgument("localization_launch_command", default_value=""),
            LogInfo(
                msg=(
                    "Waver airport full demo: ugv_world.world + ugv_rover + elevated "
                    "target(z>=3m). Ground dynamic obstacle is optional. Rover stays stopped until "
                    "the operator panel sends START_PATROL/AUTO_MODE."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(backend_launch),
                launch_arguments={
                    "use_gui": LaunchConfiguration("use_gui"),
                    "require_scan": LaunchConfiguration("require_scan"),
                    "start_gazebo": "true",
                    "spawn_robot": "true",
                    "spawn_aerial_target": "false",
                    "enable_aerial_target_pipeline": "true",
                    "target_z": LaunchConfiguration("target_z"),
                    "target_min_height_m": LaunchConfiguration("target_min_height_m"),
                    "min_dynamic_motion_m": LaunchConfiguration("min_dynamic_motion_m"),
                    "min_dynamic_velocity_mps": LaunchConfiguration("min_dynamic_velocity_mps"),
                    "spawn_ground_dynamic_obstacle": LaunchConfiguration("spawn_ground_dynamic_obstacle"),
                    "gazebo_goal_tolerance_m": LaunchConfiguration("gazebo_goal_tolerance_m"),
                    "world_file": LaunchConfiguration("world_file"),
                    "robot_sdf_file": LaunchConfiguration("robot_sdf_file"),
                    "map_yaml": LaunchConfiguration("map_yaml"),
                    "waypoint_file": LaunchConfiguration("waypoint_file"),
                    "default_mode": "STANDBY",
                }.items(),
            ),
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(panel_launch),
                        condition=IfCondition(LaunchConfiguration("use_operator_panel")),
                        launch_arguments={
                            "require_scan": LaunchConfiguration("require_scan"),
                            "map_topic": "/map",
                            "map_display_mode": "auto",
                            "global_path_topic": "/plan",
                            "local_path_topic": "/local_plan",
                            "amcl_pose_topic": "/amcl_pose",
                            "active_nav_goal_topic": "/waver/active_nav_goal",
                            "object_mission_goal_topic": "/waver/object_mission_goal",
                            "lidar_objects_map_topic": "/waver/lidar_objects_map",
                            "elevated_dynamic_target_topic": "/waver/elevated_dynamic_targets",
                            "mission_state_topic": "/waver/mission_state",
                            "patrol_status_topic": "/waver/patrol_status",
                            "safety_state_topic": "/waver/safety_state",
                            "sound_mission_status_topic": "/waver/sound_mission_status",
                            "camera_detection_status_topic": "/waver/camera_detection_state",
                            "gazebo_trial_state_topic": "/waver/gazebo_trial_target_state",
                            "publish_direct_cmd_vel": "false",
                            "auto_mode_strategy": "mission_nav2",
                            "mapping_launch_command": LaunchConfiguration("mapping_launch_command"),
                            "map_save_command": LaunchConfiguration("map_save_command"),
                            "localization_launch_command": LaunchConfiguration("localization_launch_command"),
                        }.items(),
                    )
                ],
            ),
        ]
    )
