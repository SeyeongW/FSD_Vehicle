from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share_path(*parts: str) -> str:
    from ament_index_python.packages import get_package_share_directory

    return os.path.join(get_package_share_directory("waver_patrol"), *parts)


def generate_launch_description() -> LaunchDescription:
    default_extrinsic = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/config/sensors/camera_lidar_extrinsic.yaml")
    default_profile = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/config/real_profiles/bird_patrol_production.yaml")
    default_feedback_schema = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/config/waver_base_feedback_schema.yaml")
    default_waypoints = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/src/waver_patrol/waypoints/waver_real_0p5m_square_patrol.yaml")
    default_nav2_params = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/src/waver_patrol/config/nav2_params_waver_real.yaml")
    default_mission_params = os.path.expanduser(
        "~/ros2_ws5/FSD_Vehicle/src/waver_patrol/config/waver_nav2_radar_bird_mission_real.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("profile", default_value=default_profile),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("real_profile", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("default_mode", default_value="STANDBY"),
            DeclareLaunchArgument("map", default_value=os.path.expanduser("~/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml")),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("mission_params_file", default_value=default_mission_params),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("feedback_schema_path", default_value=default_feedback_schema),
            DeclareLaunchArgument("bird_model_path", default_value=""),
            DeclareLaunchArgument("camera_lidar_extrinsic", default_value=default_extrinsic),
            DeclareLaunchArgument("camera_image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("scan_topic", default_value="/scan_safety"),
            DeclareLaunchArgument("scan_source", default_value="mid360"),
            DeclareLaunchArgument("scan_source_safety", default_value="mid360"),
            DeclareLaunchArgument("scan_source_slam", default_value="mid360"),
            DeclareLaunchArgument("odom_source", default_value="ekf"),
            DeclareLaunchArgument("enable_robot_localization", default_value="true"),
            DeclareLaunchArgument("enable_waver_base_driver", default_value="true"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("start_base_feedback", default_value="false"),
            DeclareLaunchArgument("include_existing_ugv_driver", default_value="false"),
            DeclareLaunchArgument("enable_livox_scan_adapter", default_value="true"),
            DeclareLaunchArgument("enable_pointcloud_lidar_objects", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_map_transform", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_motion_filter", default_value="true"),
            DeclareLaunchArgument("enable_target_goal_manager", default_value="true"),
            DeclareLaunchArgument("enable_target_departure_monitor", default_value="true"),
            DeclareLaunchArgument("enable_radar_command_bridge", default_value="false"),
            DeclareLaunchArgument("enable_bird_detector", default_value="true"),
            DeclareLaunchArgument("enable_bird_3d_fusion", default_value="true"),
            DeclareLaunchArgument("enable_camera_gimbal_controller", default_value="true"),
            DeclareLaunchArgument("enable_sound_deterrent", default_value="true"),
            DeclareLaunchArgument("enable_auto_behavior_mux", default_value="true"),
            DeclareLaunchArgument("enable_experiment_logger", default_value="true"),
            DeclareLaunchArgument("enable_velocity_smoother", default_value="true"),
            DeclareLaunchArgument("enable_collision_monitor", default_value="true"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("enable_sound_output", default_value="false"),
            DeclareLaunchArgument("sound_safety_ack", default_value="false"),
            DeclareLaunchArgument("safety_max_linear_speed", default_value="0.05"),
            DeclareLaunchArgument("safety_max_angular_speed", default_value="0.20"),
            LogInfo(msg="[BIRD PATROL] production wrapper: bird stack is in pipeline; unavailable capabilities must fail closed."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_share_path("launch", "sensor_static_transforms.launch.py")),
                launch_arguments={"camera_lidar_extrinsic": LaunchConfiguration("camera_lidar_extrinsic")}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(_share_path("launch", "waver_real_bird_autonomy.launch.py")),
                launch_arguments={
                    "real_profile": "true",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "use_nav2": LaunchConfiguration("use_nav2"),
                    "default_mode": LaunchConfiguration("default_mode"),
                    "enable_waver_base_driver": LaunchConfiguration("enable_waver_base_driver"),
                    "feedback_schema_path": LaunchConfiguration("feedback_schema_path"),
                    "start_serial_bridge": LaunchConfiguration("start_serial_bridge"),
                    "start_base_feedback": LaunchConfiguration("start_base_feedback"),
                    "include_existing_ugv_driver": LaunchConfiguration("include_existing_ugv_driver"),
                    "odom_source": LaunchConfiguration("odom_source"),
                    "enable_robot_localization": LaunchConfiguration("enable_robot_localization"),
                    "enable_livox_scan_adapter": LaunchConfiguration("enable_livox_scan_adapter"),
                    "enable_pointcloud_lidar_objects": LaunchConfiguration("enable_pointcloud_lidar_objects"),
                    "enable_moving_object_map_transform": LaunchConfiguration("enable_moving_object_map_transform"),
                    "enable_moving_object_motion_filter": LaunchConfiguration("enable_moving_object_motion_filter"),
                    "enable_target_goal_manager": LaunchConfiguration("enable_target_goal_manager"),
                    "enable_mission_patrol_manager": "true",
                    "enable_auto_behavior_mux": LaunchConfiguration("enable_auto_behavior_mux"),
                    "enable_bird_detector": LaunchConfiguration("enable_bird_detector"),
                    "enable_bird_3d_fusion": LaunchConfiguration("enable_bird_3d_fusion"),
                    "enable_camera_gimbal_controller": LaunchConfiguration("enable_camera_gimbal_controller"),
                    "enable_target_departure_monitor": LaunchConfiguration("enable_target_departure_monitor"),
                    "enable_radar_command_bridge": LaunchConfiguration("enable_radar_command_bridge"),
                    "enable_sound_deterrent": LaunchConfiguration("enable_sound_deterrent"),
                    "enable_sound_output": LaunchConfiguration("enable_sound_output"),
                    "sound_safety_ack": LaunchConfiguration("sound_safety_ack"),
                    "enable_test_publishers": "false",
                    "enable_deep_learning_stub": "false",
                    "enable_experiment_logger": LaunchConfiguration("enable_experiment_logger"),
                    "enable_velocity_smoother": LaunchConfiguration("enable_velocity_smoother"),
                    "enable_collision_monitor": LaunchConfiguration("enable_collision_monitor"),
                    "require_scan": LaunchConfiguration("require_scan"),
                    "map": LaunchConfiguration("map"),
                    "waypoint_file": LaunchConfiguration("waypoint_file"),
                    "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    "mission_params_file": LaunchConfiguration("mission_params_file"),
                    "serial_port": LaunchConfiguration("serial_port"),
                    "bird_model_path": LaunchConfiguration("bird_model_path"),
                    "camera_lidar_extrinsic": LaunchConfiguration("camera_lidar_extrinsic"),
                    "camera_image_topic": LaunchConfiguration("camera_image_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "scan_source": LaunchConfiguration("scan_source"),
                    "scan_source_safety": LaunchConfiguration("scan_source_safety"),
                    "scan_source_slam": LaunchConfiguration("scan_source_slam"),
                    "safety_max_linear_speed": LaunchConfiguration("safety_max_linear_speed"),
                    "safety_max_angular_speed": LaunchConfiguration("safety_max_angular_speed"),
                }.items(),
            ),
            Node(
                package="waver_patrol",
                executable="bird_mission_supervisor_node",
                name="bird_mission_supervisor_node",
                output="screen",
                parameters=[
                    {
                        "profile_path": LaunchConfiguration("profile"),
                        "zero_cmd_topic": "/waver/manual_cmd_vel",
                        "state_topic": "/waver/bird_mission_supervisor_state",
                    }
                ],
            ),
        ]
    )
