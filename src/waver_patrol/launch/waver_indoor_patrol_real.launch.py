from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("waver_patrol")
    real_launch = os.path.join(share, "launch", "waver_real_bird_autonomy.launch.py")
    default_map = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml")
    default_waypoints = os.path.join(share, "waypoints", "waver_real_0p5m_square_patrol.yaml")
    default_nav2_params = os.path.join(share, "config", "nav2_params_waver_real.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("default_mode", default_value="STANDBY"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("enable_waver_base_driver", default_value="false"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("start_base_feedback", default_value="false"),
            DeclareLaunchArgument("include_existing_ugv_driver", default_value="false"),
            DeclareLaunchArgument("feedback_serial_port", default_value=""),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("scan_source_safety", default_value="mid360"),
            DeclareLaunchArgument("scan_source_slam", default_value="mid360"),
            DeclareLaunchArgument("odom_source", default_value="ekf"),
            DeclareLaunchArgument("safety_max_linear_speed", default_value="0.05"),
            DeclareLaunchArgument("safety_max_angular_speed", default_value="0.20"),
            DeclareLaunchArgument("enable_livox_scan_adapter", default_value="true"),
            DeclareLaunchArgument("enable_velocity_smoother", default_value="true"),
            DeclareLaunchArgument("enable_battery_return", default_value="false"),
            DeclareLaunchArgument("enable_robot_localization", default_value="true"),
            LogInfo(
                msg=(
                    "[WAVER INDOOR REAL] Safe low-speed patrol profile: STANDBY, "
                    "scan required, final /cmd_vel from safety_cmd_mux_node only."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(real_launch),
                launch_arguments={
                    "real_profile": "true",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "use_nav2": LaunchConfiguration("use_nav2"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "default_mode": LaunchConfiguration("default_mode"),
                    "map": LaunchConfiguration("map"),
                    "waypoint_file": LaunchConfiguration("waypoint_file"),
                    "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    "serial_port": LaunchConfiguration("serial_port"),
                    "enable_waver_base_driver": LaunchConfiguration("enable_waver_base_driver"),
                    "start_serial_bridge": LaunchConfiguration("start_serial_bridge"),
                    "start_base_feedback": LaunchConfiguration("start_base_feedback"),
                    "feedback_serial_port": LaunchConfiguration("feedback_serial_port"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                    "scan_source_safety": LaunchConfiguration("scan_source_safety"),
                    "scan_source_slam": LaunchConfiguration("scan_source_slam"),
                    "scan_source": LaunchConfiguration("scan_source_safety"),
                    "odom_source": LaunchConfiguration("odom_source"),
                    "safety_max_linear_speed": LaunchConfiguration("safety_max_linear_speed"),
                    "safety_max_angular_speed": LaunchConfiguration("safety_max_angular_speed"),
                    "enable_livox_scan_adapter": LaunchConfiguration("enable_livox_scan_adapter"),
                    "enable_robot_localization": LaunchConfiguration("enable_robot_localization"),
                    "require_scan": "true",
                    "enable_test_publishers": "false",
                    "enable_deep_learning_stub": "false",
                    "include_existing_ugv_driver": LaunchConfiguration("include_existing_ugv_driver"),
                    "enable_bird_detector": "false",
                    "enable_bird_3d_fusion": "false",
                    "enable_camera_gimbal_controller": "false",
                    "enable_sound_deterrent": "false",
                    "enable_sound_output": "false",
                    "sound_safety_ack": "false",
                    "enable_target_departure_monitor": "false",
                    "enable_radar_command_bridge": "false",
                    "enable_target_goal_manager": "false",
                    "enable_mission_patrol_manager": "true",
                    "enable_auto_behavior_mux": "false",
                    "enable_experiment_logger": "false",
                    "enable_battery_return": LaunchConfiguration("enable_battery_return"),
                    "enable_pointcloud_lidar_objects": "false",
                    "enable_moving_object_map_transform": "false",
                    "enable_moving_object_motion_filter": "false",
                    "enable_velocity_smoother": LaunchConfiguration("enable_velocity_smoother"),
                    "enable_collision_monitor": "false",
                    "bird_model_path": "",
                    "experiment_name": "waver_indoor_patrol_real",
                }.items(),
            ),
        ]
    )
