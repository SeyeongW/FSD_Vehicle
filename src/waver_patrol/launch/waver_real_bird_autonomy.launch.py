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
    scan_source = value("scan_source_safety") or value("scan_source")
    if scan_source not in ("mid360", "ldlidar"):
        raise RuntimeError("waver_real_bird_autonomy: scan_source_safety/scan_source must be mid360 or ldlidar")
    if value("odom_source") not in ("ekf", "base", "rf2o"):
        raise RuntimeError("waver_real_bird_autonomy: odom_source must be ekf, base, or rf2o")
    if value("start_serial_bridge").lower() == "true" and not value("serial_port"):
        raise RuntimeError(
            "waver_real_bird_autonomy: start_serial_bridge=true requires serial_port, "
            "prefer /dev/serial/by-id/<WAVER_SERIAL_ID>"
        )
    if value("enable_waver_base_driver").lower() == "true":
        if not value("serial_port"):
            raise RuntimeError("waver_real_bird_autonomy: enable_waver_base_driver=true requires serial_port")
        if value("start_serial_bridge").lower() == "true" or value("start_base_feedback").lower() == "true":
            raise RuntimeError(
                "waver_real_bird_autonomy: enable_waver_base_driver=true forbids split serial nodes "
                "(start_serial_bridge/start_base_feedback must be false)"
            )
    if value("start_serial_bridge").lower() == "true" and value("start_base_feedback").lower() == "true":
        raise RuntimeError(
            "waver_real_bird_autonomy: split command and feedback serial owners are forbidden in real profile. "
            "Use enable_waver_base_driver=true for one serial owner."
        )
    if value("start_base_feedback").lower() == "true" and not value("feedback_serial_port"):
        raise RuntimeError("waver_real_bird_autonomy: start_base_feedback=true requires feedback_serial_port")
    if value("enable_sound_output").lower() == "true" and value("sound_safety_ack").lower() != "true":
        raise RuntimeError("waver_real_bird_autonomy: enable_sound_output=true requires sound_safety_ack=true")
    return []


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("waver_patrol")
    default_nav2_params = os.path.join(share, "config", "nav2_params_waver_real.yaml")
    default_mission_params = os.path.join(share, "config", "waver_nav2_radar_bird_mission_real.yaml")
    default_ekf_params = os.path.join(share, "config", "ekf_waver_real.yaml")
    default_map = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml")
    default_experiment_output_root = os.path.expanduser("~/ros2_ws5/FSD_Vehicle/experiment_results")
    default_waypoints = os.path.join(share, "waypoints", "waver_real_0p5m_square_patrol.yaml")

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
            DeclareLaunchArgument("scan_source", default_value="mid360"),
            DeclareLaunchArgument("scan_source_safety", default_value="mid360"),
            DeclareLaunchArgument("scan_source_slam", default_value="mid360"),
            DeclareLaunchArgument("odom_source", default_value="ekf"),
            DeclareLaunchArgument("enable_test_publishers", default_value="false"),
            DeclareLaunchArgument("enable_deep_learning_stub", default_value="false"),
            DeclareLaunchArgument("enable_bird_detector", default_value="false"),
            DeclareLaunchArgument("enable_bird_3d_fusion", default_value="false"),
            DeclareLaunchArgument("enable_camera_gimbal_controller", default_value="false"),
            DeclareLaunchArgument("enable_sound_deterrent", default_value="false"),
            DeclareLaunchArgument("enable_target_departure_monitor", default_value="false"),
            DeclareLaunchArgument("enable_radar_command_bridge", default_value="false"),
            DeclareLaunchArgument("enable_target_goal_manager", default_value="false"),
            DeclareLaunchArgument("enable_mission_patrol_manager", default_value="true"),
            DeclareLaunchArgument("enable_auto_behavior_mux", default_value="true"),
            DeclareLaunchArgument("enable_experiment_logger", default_value="false"),
            DeclareLaunchArgument("enable_battery_return", default_value="true"),
            DeclareLaunchArgument("experiment_name", default_value="waver_lidar_first_bird_deterrence"),
            DeclareLaunchArgument("experiment_output_root", default_value=default_experiment_output_root),
            DeclareLaunchArgument("enable_sound_output", default_value="false"),
            DeclareLaunchArgument("sound_safety_ack", default_value="false"),
            DeclareLaunchArgument("enable_robot_localization", default_value="true"),
            DeclareLaunchArgument("enable_waver_base_driver", default_value="false"),
            DeclareLaunchArgument("enable_legacy_ugv_base_odometry_node", default_value="false"),
            DeclareLaunchArgument("start_base_feedback", default_value="false"),
            DeclareLaunchArgument("feedback_serial_port", default_value=""),
            DeclareLaunchArgument("feedback_baudrate", default_value="115200"),
            DeclareLaunchArgument("enable_livox_scan_adapter", default_value="true"),
            DeclareLaunchArgument("enable_ldlidar", default_value="false"),
            DeclareLaunchArgument("enable_rf2o", default_value="false"),
            DeclareLaunchArgument("enable_pointcloud_lidar_objects", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_map_transform", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_motion_filter", default_value="true"),
            DeclareLaunchArgument("enable_velocity_smoother", default_value="true"),
            DeclareLaunchArgument("enable_collision_monitor", default_value="false"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("include_existing_ugv_driver", default_value="false"),
            DeclareLaunchArgument("default_mode", default_value="STANDBY"),
            DeclareLaunchArgument("safety_max_linear_speed", default_value="0.05"),
            DeclareLaunchArgument("safety_max_angular_speed", default_value="0.20"),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("camera_image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/livox/lidar"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("ekf_params_file", default_value=default_ekf_params),
            DeclareLaunchArgument("mission_params_file", default_value=default_mission_params),
            DeclareLaunchArgument("bird_model_path", default_value=""),
            DeclareLaunchArgument("bird_backend", default_value="yolo"),
            DeclareLaunchArgument("bird_confidence_threshold", default_value="0.65"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            OpaqueFunction(function=_validate_real_profile),
            LogInfo(msg="[WAVER REAL] Branch must be jo. Check before wheel-on: git branch --show-current"),
            LogInfo(
                msg=(
                    "[WAVER REAL] Command chain: Nav2 -> /waver/cmd_vel_nav2_raw -> velocity_smoother "
                    "-> /waver/cmd_vel_nav2_smooth -> safety_cmd_mux_node -> /cmd_vel -> canonical serial/base driver"
                )
            ),
            LogInfo(msg="[WAVER REAL] Test publishers disabled; default mode STANDBY; first wheel-on speed cap 0.05 m/s, 0.20 rad/s."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "waver_nav2_radar_bird_mission.launch.py")),
                launch_arguments={
                    "config_file": LaunchConfiguration("mission_params_file"),
                    "use_nav2": LaunchConfiguration("use_nav2"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    "map": LaunchConfiguration("map"),
                    "waypoint_file": LaunchConfiguration("waypoint_file"),
                    "use_localization": "amcl",
                    "use_localplan": "dwa",
                    "enable_deep_learning_stub": "false",
                    "enable_sound_stub": "false",
                    "enable_sound_deterrent": LaunchConfiguration("enable_sound_deterrent"),
                    "enable_camera_gimbal_controller": LaunchConfiguration("enable_camera_gimbal_controller"),
                    "enable_target_departure_monitor": LaunchConfiguration("enable_target_departure_monitor"),
                    "enable_radar_command_bridge": LaunchConfiguration("enable_radar_command_bridge"),
                    "enable_target_goal_manager": LaunchConfiguration("enable_target_goal_manager"),
                    "enable_mission_patrol_manager": LaunchConfiguration("enable_mission_patrol_manager"),
                    "enable_auto_behavior_mux": LaunchConfiguration("enable_auto_behavior_mux"),
                    "enable_experiment_logger": LaunchConfiguration("enable_experiment_logger"),
                    "enable_battery_return": LaunchConfiguration("enable_battery_return"),
                    "experiment_name": LaunchConfiguration("experiment_name"),
                    "experiment_output_root": LaunchConfiguration("experiment_output_root"),
                    "enable_sound_output": LaunchConfiguration("enable_sound_output"),
                    "sound_safety_ack": LaunchConfiguration("sound_safety_ack"),
                    "enable_test_publishers": "false",
                    "enable_livox_scan_adapter": PythonExpression(
                        [
                            "'true' if '",
                            LaunchConfiguration("enable_livox_scan_adapter"),
                            "' == 'true' and '",
                            LaunchConfiguration("scan_source_safety"),
                            "' == 'mid360' else 'false'",
                        ]
                    ),
                    "enable_pointcloud_lidar_objects": LaunchConfiguration("enable_pointcloud_lidar_objects"),
                    "enable_moving_object_map_transform": LaunchConfiguration("enable_moving_object_map_transform"),
                    "enable_moving_object_motion_filter": LaunchConfiguration("enable_moving_object_motion_filter"),
                    "start_serial_bridge": LaunchConfiguration("start_serial_bridge"),
                    "start_base_feedback": LaunchConfiguration("start_base_feedback"),
                    "feedback_serial_port": LaunchConfiguration("feedback_serial_port"),
                    "feedback_baudrate": LaunchConfiguration("feedback_baudrate"),
                    "base_node_executable": PythonExpression(
                        ["'base_node_ekf' if '", LaunchConfiguration("odom_source"), "' == 'ekf' else 'base_node'"]
                    ),
                    "enable_legacy_ugv_base_odometry_node": PythonExpression(
                        [
                            "'false' if '",
                            LaunchConfiguration("enable_waver_base_driver"),
                            "' == 'true' else '",
                            LaunchConfiguration("enable_legacy_ugv_base_odometry_node"),
                            "'",
                        ]
                    ),
                    "pub_odom_tf": PythonExpression(
                        ["'false' if '", LaunchConfiguration("odom_source"), "' == 'ekf' else 'true'"]
                    ),
                    "enable_ldlidar": PythonExpression(
                        ["'true' if '", LaunchConfiguration("scan_source_safety"), "' == 'ldlidar' else 'false'"]
                    ),
                    "enable_rf2o": PythonExpression(
                        ["'true' if '", LaunchConfiguration("odom_source"), "' == 'rf2o' else 'false'"]
                    ),
                    "include_existing_ugv_driver": "false",
                    "serial_port": LaunchConfiguration("serial_port"),
                    "require_explicit_serial_port": "true",
                    "default_mode": LaunchConfiguration("default_mode"),
                    "require_scan": LaunchConfiguration("require_scan"),
                    "safety_max_linear_speed": LaunchConfiguration("safety_max_linear_speed"),
                    "safety_max_angular_speed": LaunchConfiguration("safety_max_angular_speed"),
                    "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "enable_velocity_smoother": LaunchConfiguration("enable_velocity_smoother"),
                    "nav2_controller_cmd_topic": PythonExpression(
                        [
                            "'/waver/cmd_vel_nav2_raw' if '",
                            LaunchConfiguration("enable_velocity_smoother"),
                            "' == 'true' else '/waver/cmd_vel_nav2'",
                        ]
                    ),
                    "velocity_smoother_input_topic": "/waver/cmd_vel_nav2_raw",
                    "velocity_smoother_output_topic": "/waver/cmd_vel_nav2_smooth",
                    "cmd_vel_auto_topic": "/waver/cmd_vel_auto",
                    "cmd_vel_target_track_topic": "/waver/cmd_vel_target_track",
                    "cmd_vel_return_home_topic": "/waver/cmd_vel_return_home",
                    "auto_behavior_target_active_topic": "/waver/inspection_target_active",
                    "safety_nav2_cmd_topic": PythonExpression(
                        [
                            "'/waver/cmd_vel_nav2_smooth' if '",
                            LaunchConfiguration("enable_velocity_smoother"),
                            "' == 'true' else '/waver/cmd_vel_nav2'",
                        ]
                    ),
                    "remap_nav2_cmd_vel": "true",
                    "use_rviz": LaunchConfiguration("use_rviz"),
                }.items(),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("enable_robot_localization"),
                            "' == 'true' and '",
                            LaunchConfiguration("odom_source"),
                            "' == 'ekf'",
                        ]
                    )
                ),
                parameters=[
                    LaunchConfiguration("ekf_params_file"),
                    {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
                ],
            ),
            Node(
                package="waver_patrol",
                executable="waver_base_driver_node",
                name="waver_base_driver_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_waver_base_driver")),
                parameters=[
                    *common,
                    {
                        "serial_port": ParameterValue(LaunchConfiguration("serial_port"), value_type=str),
                        "cmd_vel_topic": "/cmd_vel",
                        "odom_topic": PythonExpression(
                            ["'/odom_raw' if '", LaunchConfiguration("odom_source"), "' == 'ekf' else '/odom'"]
                        ),
                        "publish_tf": PythonExpression(
                            ["'false' if '", LaunchConfiguration("odom_source"), "' == 'ekf' else 'true'"]
                        ),
                        "publish_legacy_float32_odom_raw": False,
                        "command_protocol": "lr",
                        "cmd_timeout_s": 0.3,
                        "angular_gain": 0.55,
                        "max_left_right": 0.32,
                        "max_demo_speed": 0.32,
                        "min_linear_ratio": 0.25,
                        "wheel_delta_per_tick": 0.05,
                        "pure_turn_mode": "pivot",
                        "pure_turn_min_ratio": 0.16,
                        "pure_turn_max_ratio": 0.16,
                        "min_motor_voltage_v": 7.0,
                        "mixed_turn_mode": "inside_brake",
                        "mixed_turn_inner_ratio": 0.0,
                        "mixed_turn_outer_ratio": 0.22,
                    },
                ],
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
