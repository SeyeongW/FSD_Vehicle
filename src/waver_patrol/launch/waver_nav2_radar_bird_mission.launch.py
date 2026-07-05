from __future__ import annotations

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("waver_patrol")
    default_config = os.path.join(share, "config", "waver_nav2_radar_bird_mission.yaml")
    default_waypoints = os.path.join(share, "waypoints", "waver_nav2_patrol_mission.yaml")
    default_nav2_params = os.path.join(share, "config", "nav2_params_waver.yaml")
    default_collision_params = os.path.join(share, "config", "collision_monitor_waver.yaml")
    try:
        default_map = os.path.join(get_package_share_directory("ugv_nav"), "maps", "map.yaml")
    except PackageNotFoundError:
        default_map = os.path.join(share, "maps", "map.yaml")

    config_file = LaunchConfiguration("config_file")
    waypoint_file = LaunchConfiguration("waypoint_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    require_scan = LaunchConfiguration("require_scan")
    default_mode = LaunchConfiguration("default_mode")
    use_nav2 = LaunchConfiguration("use_nav2")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    nav2_controller_cmd_topic = LaunchConfiguration("nav2_controller_cmd_topic")
    velocity_smoother_input_topic = LaunchConfiguration("velocity_smoother_input_topic")
    velocity_smoother_output_topic = LaunchConfiguration("velocity_smoother_output_topic")
    safety_nav2_cmd_topic = LaunchConfiguration("safety_nav2_cmd_topic")
    safety_cmd_vel_out_topic = LaunchConfiguration("safety_cmd_vel_out_topic")
    collision_cmd_vel_in_topic = LaunchConfiguration("collision_cmd_vel_in_topic")
    collision_cmd_vel_out_topic = LaunchConfiguration("collision_cmd_vel_out_topic")
    start_base_feedback = LaunchConfiguration("start_base_feedback")
    feedback_serial_port = LaunchConfiguration("feedback_serial_port")
    feedback_baudrate = LaunchConfiguration("feedback_baudrate")
    base_node_executable = LaunchConfiguration("base_node_executable")
    enable_legacy_ugv_base_odometry_node = LaunchConfiguration("enable_legacy_ugv_base_odometry_node")
    pub_odom_tf = LaunchConfiguration("pub_odom_tf")
    enable_ldlidar = LaunchConfiguration("enable_ldlidar")
    enable_rf2o = LaunchConfiguration("enable_rf2o")
    enable_auto_behavior_mux = LaunchConfiguration("enable_auto_behavior_mux")
    cmd_vel_auto_topic = LaunchConfiguration("cmd_vel_auto_topic")
    cmd_vel_target_track_topic = LaunchConfiguration("cmd_vel_target_track_topic")
    cmd_vel_return_home_topic = LaunchConfiguration("cmd_vel_return_home_topic")
    auto_behavior_target_active_topic = LaunchConfiguration("auto_behavior_target_active_topic")

    common = [config_file, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]
    serial_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("start_serial_bridge"),
                "' == 'true' and '",
                LaunchConfiguration("include_existing_ugv_driver"),
                "' == 'false'",
            ]
        )
    )

    ugv_nav_include = GroupAction(
        condition=IfCondition(use_nav2),
        actions=[
            LogInfo(msg="Nav2 enabled: remapping Nav2 controller output to the configured safety candidate topic."),
            SetRemap(src="/cmd_vel", dst=nav2_controller_cmd_topic, condition=IfCondition(LaunchConfiguration("remap_nav2_cmd_vel"))),
            SetRemap(src="cmd_vel", dst=nav2_controller_cmd_topic, condition=IfCondition(LaunchConfiguration("remap_nav2_cmd_vel"))),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("ugv_nav"), "/launch/nav.launch.py"]
                ),
                launch_arguments={
                    "use_localization": LaunchConfiguration("use_localization"),
                    "use_localplan": LaunchConfiguration("use_localplan"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "map": LaunchConfiguration("map"),
                    "params_file": LaunchConfiguration("nav2_params_file"),
                    "pub_odom_tf": pub_odom_tf,
                    "start_base_feedback": start_base_feedback,
                    "feedback_serial_port": feedback_serial_port,
                    "feedback_baudrate": feedback_baudrate,
                    "base_node_executable": base_node_executable,
                    "enable_legacy_ugv_base_odometry_node": enable_legacy_ugv_base_odometry_node,
                    "enable_ldlidar": enable_ldlidar,
                    "enable_rf2o": enable_rf2o,
                }.items(),
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoints),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("nav2_params_file", default_value=default_nav2_params),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("use_localization", default_value="amcl"),
            DeclareLaunchArgument("use_amcl", default_value="true"),
            DeclareLaunchArgument("use_slam", default_value="false"),
            DeclareLaunchArgument("use_localplan", default_value="dwa"),
            DeclareLaunchArgument("enable_mission_patrol_manager", default_value="true"),
            DeclareLaunchArgument("enable_radar_command_bridge", default_value="true"),
            DeclareLaunchArgument("enable_target_goal_manager", default_value="true"),
            DeclareLaunchArgument("enable_safety_cmd_mux", default_value="true"),
            DeclareLaunchArgument("enable_velocity_smoother", default_value="false"),
            DeclareLaunchArgument("enable_battery_return", default_value="true"),
            DeclareLaunchArgument("enable_deep_learning_stub", default_value="false"),
            DeclareLaunchArgument("enable_sound_stub", default_value="false"),
            DeclareLaunchArgument("enable_sound_deterrent", default_value="true"),
            DeclareLaunchArgument("enable_camera_gimbal_controller", default_value="true"),
            DeclareLaunchArgument("enable_target_departure_monitor", default_value="true"),
            DeclareLaunchArgument("enable_auto_behavior_mux", default_value="true"),
            DeclareLaunchArgument("enable_experiment_logger", default_value="true"),
            DeclareLaunchArgument("experiment_name", default_value="waver_lidar_first_bird_deterrence"),
            DeclareLaunchArgument("experiment_output_root", default_value="$HOME/ros2_ws5/FSD_Vehicle/experiment_results"),
            DeclareLaunchArgument("enable_keyboard_teleop", default_value="false"),
            DeclareLaunchArgument("enable_test_publishers", default_value="false"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("include_existing_ugv_driver", default_value="false"),
            DeclareLaunchArgument("serial_port", default_value=""),
            DeclareLaunchArgument("require_explicit_serial_port", default_value="false"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("ignore_scan_when_require_scan_false", default_value="false"),
            DeclareLaunchArgument("safety_max_linear_speed", default_value="0.18"),
            DeclareLaunchArgument("safety_max_angular_speed", default_value="0.45"),
            DeclareLaunchArgument("default_mode", default_value="STANDBY"),
            DeclareLaunchArgument("target_classification_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("mission_sound_task_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("sound_task_duration_sec", default_value="5.0"),
            DeclareLaunchArgument("sound_alert_cooldown_sec", default_value="10.0"),
            DeclareLaunchArgument("sound_done_latch_sec", default_value="1.2"),
            DeclareLaunchArgument("enable_sound_output", default_value="false"),
            DeclareLaunchArgument("sound_safety_ack", default_value="false"),
            DeclareLaunchArgument("post_target_resume_cooldown_sec", default_value="8.0"),
            DeclareLaunchArgument("enable_sim_nav_goal_arrival", default_value="false"),
            DeclareLaunchArgument("enable_livox_scan_adapter", default_value="false"),
            DeclareLaunchArgument("enable_pointcloud_lidar_objects", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_map_transform", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_motion_filter", default_value="true"),
            DeclareLaunchArgument("enable_moving_object_goal_relay", default_value="false"),
            DeclareLaunchArgument("require_robot_pose_for_goal", default_value="true"),
            DeclareLaunchArgument("start_base_feedback", default_value="false"),
            DeclareLaunchArgument("feedback_serial_port", default_value=""),
            DeclareLaunchArgument("feedback_baudrate", default_value="115200"),
            DeclareLaunchArgument("base_node_executable", default_value="base_node"),
            DeclareLaunchArgument("enable_legacy_ugv_base_odometry_node", default_value="true"),
            DeclareLaunchArgument("pub_odom_tf", default_value="true"),
            DeclareLaunchArgument("enable_ldlidar", default_value="false"),
            DeclareLaunchArgument("enable_rf2o", default_value="false"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/mid360_PointCloud2"),
            DeclareLaunchArgument("pointcloud_target_frame", default_value="base_link"),
            DeclareLaunchArgument("pointcloud_forward_axis", default_value="x"),
            DeclareLaunchArgument("pointcloud_lateral_axis", default_value="y"),
            DeclareLaunchArgument("pointcloud_height_axis", default_value="z"),
            DeclareLaunchArgument("pointcloud_positive_lateral_is_left", default_value="true"),
            DeclareLaunchArgument("nav2_controller_cmd_topic", default_value="/waver/cmd_vel_nav2"),
            DeclareLaunchArgument("velocity_smoother_input_topic", default_value="/waver/cmd_vel_nav2_raw"),
            DeclareLaunchArgument("velocity_smoother_output_topic", default_value="/waver/cmd_vel_nav2_smooth"),
            DeclareLaunchArgument("safety_nav2_cmd_topic", default_value="/waver/cmd_vel_nav2"),
            DeclareLaunchArgument("safety_cmd_vel_out_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("enable_collision_monitor", default_value="false"),
            DeclareLaunchArgument("collision_monitor_params_file", default_value=default_collision_params),
            DeclareLaunchArgument("collision_cmd_vel_in_topic", default_value="/waver/cmd_vel_safety"),
            DeclareLaunchArgument("collision_cmd_vel_out_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("cmd_vel_auto_topic", default_value="/waver/cmd_vel_auto"),
            DeclareLaunchArgument("cmd_vel_target_track_topic", default_value="/waver/cmd_vel_target_track"),
            DeclareLaunchArgument("cmd_vel_return_home_topic", default_value="/waver/cmd_vel_return_home"),
            DeclareLaunchArgument("auto_behavior_target_active_topic", default_value="/waver/inspection_target_active"),
            DeclareLaunchArgument("moving_object_input_topic", default_value="/waver/lidar_objects"),
            DeclareLaunchArgument("moving_object_input_type", default_value="pose_array"),
            DeclareLaunchArgument("moving_object_output_topic", default_value="/waver/lidar_objects_map"),
            DeclareLaunchArgument("moving_object_marker_topic", default_value="/waver/moving_objects_map_marker"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("scan_forward_axis", default_value="x"),
            DeclareLaunchArgument("scan_lateral_axis", default_value="y"),
            DeclareLaunchArgument("scan_height_axis", default_value="z"),
            DeclareLaunchArgument("scan_positive_lateral_is_left", default_value="true"),
            DeclareLaunchArgument("remap_nav2_cmd_vel", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            LogInfo(
                msg=(
                    "Safety rule: final /cmd_vel must have exactly one publisher. "
                    "Without collision monitor it is safety_cmd_mux_node; with collision monitor it is nav2_collision_monitor."
                )
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("start_serial_bridge")),
                msg="start_serial_bridge=true: stop ugv_driver/app.py/other serial bridges first.",
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_livox_scan_adapter")),
                msg=(
                    "3D LiDAR safety adapter enabled: PointCloud2 is projected to /scan for ground-obstacle "
                    "stop/slowdown only. 2D LaserScan has no z/height information."
                ),
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_pointcloud_lidar_objects")),
                msg=(
                    "3D LiDAR object candidate adapter enabled: PointCloud2 centers are published to "
                    "/waver/lidar_objects and never directly to /cmd_vel."
                ),
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_moving_object_map_transform")),
                msg=(
                    "Moving object map transform enabled: LiDAR object coordinates are transformed to map/odom "
                    "data topics. Radar mission command topics remain separate."
                ),
            ),
            LogInfo(
                condition=IfCondition(LaunchConfiguration("enable_moving_object_motion_filter")),
                msg=(
                    "Height-based dynamic target filter enabled: only z-valid objects with height>=3m and "
                    "map/odom-compensated motion are allowed to trigger object missions."
                ),
            ),
            ugv_nav_include,
            Node(
                package="waver_patrol",
                executable="livox_pointcloud_to_scan_node",
                name="livox_pointcloud_to_scan_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_livox_scan_adapter")),
                parameters=[
                    *common,
                    {
                        "pointcloud_topic": pointcloud_topic,
                        "scan_topic": scan_topic,
                        "forward_axis": ParameterValue(LaunchConfiguration("scan_forward_axis"), value_type=str),
                        "lateral_axis": ParameterValue(LaunchConfiguration("scan_lateral_axis"), value_type=str),
                        "height_axis": ParameterValue(LaunchConfiguration("scan_height_axis"), value_type=str),
                        "positive_lateral_is_left": ParameterValue(
                            LaunchConfiguration("scan_positive_lateral_is_left"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="pointcloud_lidar_objects_node",
                name="pointcloud_lidar_objects_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_pointcloud_lidar_objects")),
                parameters=[
                    *common,
                    {
                        "pointcloud_topic": pointcloud_topic,
                        "output_pose_array_topic": "/waver/lidar_objects",
                        "target_frame": ParameterValue(LaunchConfiguration("pointcloud_target_frame"), value_type=str),
                        "forward_axis": ParameterValue(LaunchConfiguration("pointcloud_forward_axis"), value_type=str),
                        "lateral_axis": ParameterValue(LaunchConfiguration("pointcloud_lateral_axis"), value_type=str),
                        "height_axis": ParameterValue(LaunchConfiguration("pointcloud_height_axis"), value_type=str),
                        "positive_lateral_is_left": ParameterValue(
                            LaunchConfiguration("pointcloud_positive_lateral_is_left"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="moving_object_map_transform_node",
                name="moving_object_map_transform_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_moving_object_map_transform")),
                parameters=[
                    *common,
                    {
                        "input_topic": ParameterValue(LaunchConfiguration("moving_object_input_topic"), value_type=str),
                        "input_type": ParameterValue(LaunchConfiguration("moving_object_input_type"), value_type=str),
                        "output_pose_array_topic": ParameterValue(LaunchConfiguration("moving_object_output_topic"), value_type=str),
                        "debug_marker_topic": ParameterValue(LaunchConfiguration("moving_object_marker_topic"), value_type=str),
                        "target_frame": "map",
                        "fallback_frame": "odom",
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="moving_object_motion_filter_node",
                name="moving_object_motion_filter_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_moving_object_motion_filter")),
                parameters=[
                    *common,
                    {
                        "input_topic": ParameterValue(
                            LaunchConfiguration("moving_object_output_topic"),
                            value_type=str,
                        ),
                        "raw_input_topic": ParameterValue(
                            LaunchConfiguration("moving_object_input_topic"),
                            value_type=str,
                        ),
                        "target_min_height_m": 3.0,
                        "min_dynamic_motion_m": 0.2,
                        "min_dynamic_velocity_mps": 0.05,
                        "min_sample_motion_epsilon_m": 0.001,
                        "lock_on_first_valid_target": True,
                        "locked_target_gate_m": 1.0,
                        "locked_target_lost_timeout_sec": 3.0,
                        "clear_locked_target_when_static_sec": 8.0,
                        "target_point_topic": "/waver/aerial_target",
                        "target_active_topic": "/waver/aerial_target_active",
                        "elevated_targets_topic": "/waver/elevated_dynamic_targets",
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="moving_object_goal_relay_node",
                name="moving_object_goal_relay_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_moving_object_goal_relay")),
                parameters=[
                    *common,
                    {
                        "input_pose_array_topic": ParameterValue(LaunchConfiguration("moving_object_output_topic"), value_type=str),
                    },
                ],
            ),
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_velocity_smoother")),
                parameters=[
                    LaunchConfiguration("nav2_params_file"),
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
                remappings=[
                    ("cmd_vel", velocity_smoother_input_topic),
                    ("cmd_vel_smoothed", velocity_smoother_output_topic),
                ],
            ),
            Node(
                package="waver_patrol",
                executable="radar_command_bridge_node",
                name="radar_command_bridge_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_radar_command_bridge")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="target_goal_manager_node",
                name="target_goal_manager_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_target_goal_manager")),
                parameters=[
                    *common,
                    {
                        "require_robot_pose_for_goal": ParameterValue(
                            LaunchConfiguration("require_robot_pose_for_goal"),
                            value_type=bool,
                        ),
                        "max_inspection_retries_per_target": 1,
                        "inspected_target_lockout_sec": 120.0,
                        "inspected_target_radius_m": 1.5,
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="mission_patrol_manager_node",
                name="mission_patrol_manager_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_mission_patrol_manager")),
                parameters=[
                    *common,
                    {
                        "waypoint_file": waypoint_file,
                        "use_nav2": ParameterValue(use_nav2, value_type=bool),
                        "default_mode": default_mode,
                        "target_classification_timeout_sec": ParameterValue(
                            LaunchConfiguration("target_classification_timeout_sec"),
                            value_type=float,
                        ),
                        "sound_task_timeout_sec": ParameterValue(
                            LaunchConfiguration("mission_sound_task_timeout_sec"),
                            value_type=float,
                        ),
                        "post_target_resume_cooldown_sec": ParameterValue(
                            LaunchConfiguration("post_target_resume_cooldown_sec"),
                            value_type=float,
                        ),
                        "enable_sim_nav_goal_arrival": ParameterValue(
                            LaunchConfiguration("enable_sim_nav_goal_arrival"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="camera_gimbal_controller_node",
                name="camera_gimbal_controller_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_camera_gimbal_controller")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="target_departure_monitor_node",
                name="target_departure_monitor_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_target_departure_monitor")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="battery_return_manager_node",
                name="battery_return_manager_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_battery_return")),
                parameters=[*common, {"waypoint_file": waypoint_file}],
            ),
            Node(
                package="waver_patrol",
                executable="deep_learning_bridge_stub",
                name="deep_learning_bridge_stub",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_deep_learning_stub")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="sound_alert_stub",
                name="sound_alert_stub",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_sound_stub")),
                parameters=[
                    *common,
                    {
                        "sound_task_duration_sec": ParameterValue(
                            LaunchConfiguration("sound_task_duration_sec"),
                            value_type=float,
                        ),
                        "alert_cooldown_sec": ParameterValue(
                            LaunchConfiguration("sound_alert_cooldown_sec"),
                            value_type=float,
                        ),
                        "done_latch_sec": ParameterValue(
                            LaunchConfiguration("sound_done_latch_sec"),
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="sound_deterrent_node",
                name="sound_deterrent_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_sound_deterrent")),
                parameters=[
                    *common,
                    {
                        "enable_sound_output": ParameterValue(
                            LaunchConfiguration("enable_sound_output"),
                            value_type=bool,
                        ),
                        "sound_task_duration_sec": ParameterValue(
                            LaunchConfiguration("sound_task_duration_sec"),
                            value_type=float,
                        ),
                        "alert_cooldown_sec": ParameterValue(
                            LaunchConfiguration("sound_alert_cooldown_sec"),
                            value_type=float,
                        ),
                        "safety_ack": ParameterValue(
                            LaunchConfiguration("sound_safety_ack"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="auto_behavior_mux_node",
                name="auto_behavior_mux_node",
                output="screen",
                condition=IfCondition(enable_auto_behavior_mux),
                parameters=[
                    *common,
                    {
                        "cmd_vel_patrol_topic": safety_nav2_cmd_topic,
                        "cmd_vel_target_track_topic": cmd_vel_target_track_topic,
                        "cmd_vel_return_home_topic": cmd_vel_return_home_topic,
                        "cmd_vel_auto_topic": cmd_vel_auto_topic,
                        "aerial_target_active_topic": auto_behavior_target_active_topic,
                        "mode_topic": "/waver/mode",
                        "default_mode": default_mode,
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="safety_cmd_mux_node",
                name="safety_cmd_mux_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_safety_cmd_mux")),
                parameters=[
                    *common,
                    {
                        "require_scan": ParameterValue(require_scan, value_type=bool),
                        "ignore_scan_when_require_scan_false": ParameterValue(
                            LaunchConfiguration("ignore_scan_when_require_scan_false"),
                            value_type=bool,
                        ),
                        "max_linear_speed": ParameterValue(LaunchConfiguration("safety_max_linear_speed"), value_type=float),
                        "max_angular_speed": ParameterValue(LaunchConfiguration("safety_max_angular_speed"), value_type=float),
                        "mode_default": default_mode,
                        "nav2_cmd_topic": safety_nav2_cmd_topic,
                        "cmd_vel_auto_topic": PythonExpression(
                            ["'", cmd_vel_auto_topic, "' if '", enable_auto_behavior_mux, "' == 'true' else ''"]
                        ),
                        "cmd_vel_out_topic": safety_cmd_vel_out_topic,
                        "scan_topic": scan_topic,
                    },
                ],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_collision_monitor")),
                parameters=[
                    LaunchConfiguration("collision_monitor_params_file"),
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "cmd_vel_in_topic": collision_cmd_vel_in_topic,
                        "cmd_vel_out_topic": collision_cmd_vel_out_topic,
                    },
                ],
                remappings=[
                    ("cmd_vel_in", collision_cmd_vel_in_topic),
                    ("cmd_vel_out", collision_cmd_vel_out_topic),
                    ("scan", scan_topic),
                ],
            ),
            Node(
                package="waver_patrol",
                executable="experiment_data_logger_node",
                name="experiment_data_logger_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_experiment_logger")),
                parameters=[
                    *common,
                    {
                        "experiment_name": ParameterValue(LaunchConfiguration("experiment_name"), value_type=str),
                        "output_root": ParameterValue(LaunchConfiguration("experiment_output_root"), value_type=str),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="teleop_node",
                name="waver_keyboard_teleop",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_keyboard_teleop")),
            ),
            Node(
                package="waver_patrol",
                executable="serial_cmd_vel_bridge",
                name="serial_cmd_vel_bridge",
                output="screen",
                condition=serial_condition,
                parameters=[
                    *common,
                    {
                        "serial_port": ParameterValue(LaunchConfiguration("serial_port"), value_type=str),
                        "require_explicit_serial_port": ParameterValue(
                            LaunchConfiguration("require_explicit_serial_port"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="waver_patrol",
                executable="radar_target_test_publisher_node",
                name="radar_target_test_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_test_publishers")),
                parameters=common,
            ),
            Node(
                package="waver_patrol",
                executable="fake_camera_classification_test_publisher_node",
                name="fake_camera_classification_test_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_test_publishers")),
                parameters=common,
            ),
        ]
    )
