from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_real_profile(context, *args, **kwargs):
    def value(name: str) -> str:
        return context.launch_configurations.get(name, "").strip()

    real_profile = value("real_profile").lower() == "true"
    if not real_profile:
        return []
    forbidden_true = [
        "enable_test_publishers",
        "enable_deep_learning_stub",
        "include_existing_ugv_driver",
    ]
    for name in forbidden_true:
        if value(name).lower() == "true":
            raise RuntimeError(f"waver_real_bird_autonomy: {name}=true is forbidden when real_profile=true")
    if value("bird_backend").lower() == "mock_for_sim_only":
        raise RuntimeError("waver_real_bird_autonomy: mock bird backend is forbidden when real_profile=true")
    if value("start_serial_bridge").lower() == "true" and not value("serial_port"):
        raise RuntimeError(
            "waver_real_bird_autonomy: start_serial_bridge=true requires serial_port, "
            "prefer /dev/serial/by-id/<WAVER_SERIAL_ID>"
        )
    return []


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("waver_patrol")
    default_nav2_params = os.path.join(share, "config", "nav2_params_waver_real.yaml")
    default_mission_params = os.path.join(share, "config", "waver_nav2_radar_bird_mission_real.yaml")
    default_map = os.path.expanduser("~/ros2_ws/FSD_Vehicle/maps/waver_latest_map.yaml")

    common = [
        LaunchConfiguration("mission_params_file"),
        {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("real_profile", default_value="true"),
            DeclareLaunchArgument("command_chain_mode", default_value="safety_mux_final"),
            DeclareLaunchArgument("enable_test_publishers", default_value="false"),
            DeclareLaunchArgument("enable_deep_learning_stub", default_value="false"),
            DeclareLaunchArgument("enable_bird_detector", default_value="true"),
            DeclareLaunchArgument("enable_bird_3d_fusion", default_value="true"),
            DeclareLaunchArgument("enable_livox_scan_adapter", default_value="true"),
            DeclareLaunchArgument("enable_pointcloud_lidar_objects", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_map_transform", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_motion_filter", default_value="true"),
            DeclareLaunchArgument("enable_velocity_smoother", default_value="true"),
            DeclareLaunchArgument("enable_collision_monitor", default_value="false"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("include_existing_ugv_driver", default_value="false"),
            DeclareLaunchArgument("default_mode", default_value="STANDBY"),
            DeclareLaunchArgument("safety_max_linear_speed", default_value="0.10"),
            DeclareLaunchArgument("safety_max_angular_speed", default_value="0.35"),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("camera_image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/mid360_PointCloud2"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("mission_params_file", default_value=default_mission_params),
            DeclareLaunchArgument("bird_model_path", default_value=""),
            DeclareLaunchArgument("bird_backend", default_value="yolo"),
            DeclareLaunchArgument("bird_confidence_threshold", default_value="0.65"),
            OpaqueFunction(function=_validate_real_profile),
            LogInfo(msg="[WAVER REAL] Branch must be jo. Check before wheel-on: git branch --show-current"),
            LogInfo(
                msg=(
                    "[WAVER REAL] Command chain: Nav2 -> /waver/cmd_vel_nav2 -> safety_cmd_mux_node "
                    "-> /cmd_vel -> canonical serial/base driver"
                )
            ),
            LogInfo(msg="[WAVER REAL] Test publishers disabled; default mode STANDBY; wheel-on speed cap 0.10 m/s."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "waver_nav2_radar_bird_mission.launch.py")),
                launch_arguments={
                    "config_file": LaunchConfiguration("mission_params_file"),
                    "use_nav2": LaunchConfiguration("use_nav2"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    "map": LaunchConfiguration("map"),
                    "use_localization": "amcl",
                    "use_localplan": "dwa",
                    "enable_deep_learning_stub": "false",
                    "enable_sound_stub": "false",
                    "enable_test_publishers": "false",
                    "enable_livox_scan_adapter": LaunchConfiguration("enable_livox_scan_adapter"),
                    "enable_pointcloud_lidar_objects": LaunchConfiguration("enable_pointcloud_lidar_objects"),
                    "enable_moving_object_map_transform": LaunchConfiguration("enable_moving_object_map_transform"),
                    "enable_moving_object_motion_filter": LaunchConfiguration("enable_moving_object_motion_filter"),
                    "start_serial_bridge": LaunchConfiguration("start_serial_bridge"),
                    "include_existing_ugv_driver": "false",
                    "serial_port": LaunchConfiguration("serial_port"),
                    "require_explicit_serial_port": "true",
                    "default_mode": LaunchConfiguration("default_mode"),
                    "require_scan": LaunchConfiguration("require_scan"),
                    "safety_max_linear_speed": LaunchConfiguration("safety_max_linear_speed"),
                    "safety_max_angular_speed": LaunchConfiguration("safety_max_angular_speed"),
                    "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "remap_nav2_cmd_vel": "true",
                    "use_rviz": "false",
                }.items(),
            ),
            Node(
                package="waver_patrol",
                executable="bird_detector_node",
                name="bird_detector_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_bird_detector")),
                parameters=[
                    *common,
                    {
                        "real_profile": ParameterValue(LaunchConfiguration("real_profile"), value_type=bool),
                        "image_topic": ParameterValue(LaunchConfiguration("camera_image_topic"), value_type=str),
                        "camera_info_topic": ParameterValue(LaunchConfiguration("camera_info_topic"), value_type=str),
                        "backend": ParameterValue(LaunchConfiguration("bird_backend"), value_type=str),
                        "model_path": ParameterValue(LaunchConfiguration("bird_model_path"), value_type=str),
                        "confidence_threshold": ParameterValue(
                            LaunchConfiguration("bird_confidence_threshold"),
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="bird_3d_fusion_node",
                name="bird_3d_fusion_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_bird_3d_fusion")),
                parameters=[
                    *common,
                    {
                        "camera_info_topic": ParameterValue(LaunchConfiguration("camera_info_topic"), value_type=str),
                        "pointcloud_topic": ParameterValue(LaunchConfiguration("pointcloud_topic"), value_type=str),
                    },
                ],
            ),
            LogInfo(
                condition=IfCondition(
                    PythonExpression(["'", LaunchConfiguration("bird_model_path"), "' == ''"])
                ),
                msg="[WAVER REAL] bird_model_path is empty: patrol may run, but bird approach is blocked by bird_confirmed=false.",
            ),
        ]
    )
