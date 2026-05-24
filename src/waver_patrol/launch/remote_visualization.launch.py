from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    # 역할: 로컬 PC에서 RViz와 Waver operator panel을 같이 띄우는 시각화 launch다.
    waver_share = get_package_share_directory("waver_patrol")
    default_rviz = os.path.join(waver_share, "rviz", "pre_real_gazebo_validation.rviz")
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz_config", default_value=default_rviz),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_operator_panel", default_value="true"),
            DeclareLaunchArgument("require_scan", default_value="false"),
            DeclareLaunchArgument("map_display_mode", default_value="auto"),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("global_path_topic", default_value="/plan"),
            DeclareLaunchArgument("local_path_topic", default_value="/local_plan"),
            ExecuteProcess(
                cmd=["rviz2", "-d", LaunchConfiguration("rviz_config")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
            Node(
                package="ugv_tools",
                executable="waver_remote_panel",
                name="waver_remote_panel",
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_operator_panel")),
                parameters=[
                    {
                        "lidar_required": ParameterValue(LaunchConfiguration("require_scan"), value_type=bool),
                        "auto_mode_strategy": "mission_nav2",
                        "map_topic": LaunchConfiguration("map_topic"),
                        "map_display_mode": LaunchConfiguration("map_display_mode"),
                        "global_path_topic": LaunchConfiguration("global_path_topic"),
                        "local_path_topic": LaunchConfiguration("local_path_topic"),
                        "amcl_pose_topic": "/amcl_pose",
                        "active_nav_goal_topic": "/waver/active_nav_goal",
                        "object_mission_goal_topic": "/waver/object_mission_goal",
                        "lidar_objects_map_topic": "/waver/lidar_objects_map",
                        "elevated_dynamic_target_topic": "/waver/elevated_dynamic_targets",
                        "current_waypoint_topic": "/waver/current_waypoint",
                        "height_filter_debug_topic": "/waver/height_filter_debug",
                        "camera_detection_status_topic": "/waver/classification_state",
                        "sound_mission_status_topic": "/waver/sound_mission_status",
                        "gazebo_trial_state_topic": "/waver/gazebo_trial_state",
                    }
                ],
            ),
        ]
    )
