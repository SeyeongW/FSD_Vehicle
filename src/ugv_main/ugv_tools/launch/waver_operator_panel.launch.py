# Copyright 2026 Waver project contributors.
"""Operator-PC Waver visual remote panel.

역할:
  - 조작 PC나 Jetson 데스크톱에서 시각화 리모콘만 실행한다.
  - 버튼/키보드는 `/waver/manual_cmd_vel` 후보만 발행하고, AUTO 버튼은 `/waver/mode_cmd=AUTO`를 발행한다.
  - 최종 `/cmd_vel`은 Jetson backend의 `safety_cmd_mux_node` 하나만 발행해야 한다.
  - serial bridge는 절대 여기서 실행하지 않는다. 실차 serial은 Jetson backend에서만 잡는다.

운용:
  - Jetson과 조작 PC가 같은 ROS_DOMAIN_ID, 같은 네트워크에 있어야 ROS 2 DDS로 토픽이 보인다.
  - Web 기반 관측을 쓰려면 Jetson에 foxglove_bridge를 추가로 띄우고 PC 브라우저에서 접속한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 역할: 조작 PC에서 현장 상황에 맞게 바꿀 수 있는 값만 노출한다.
    ugv_tools_share = FindPackageShare("ugv_tools")
    require_scan = LaunchConfiguration("require_scan")
    min_valid_scan_points = LaunchConfiguration("min_valid_scan_points")
    control_config_file = LaunchConfiguration("control_config_file")
    waypoint_file = LaunchConfiguration("waypoint_file")
    auto_waypoints_csv = LaunchConfiguration("auto_waypoints_csv")
    max_patrol_radius_m = LaunchConfiguration("max_patrol_radius_m")
    auto_mode_strategy = LaunchConfiguration("auto_mode_strategy")
    auto_launch_command = LaunchConfiguration("auto_launch_command")
    mapping_launch_command = LaunchConfiguration("mapping_launch_command")
    map_save_command = LaunchConfiguration("map_save_command")
    localization_launch_command = LaunchConfiguration("localization_launch_command")
    map_topic = LaunchConfiguration("map_topic")
    fixed_map_topic = LaunchConfiguration("fixed_map_topic")
    map_display_mode = LaunchConfiguration("map_display_mode")
    amcl_pose_topic = LaunchConfiguration("amcl_pose_topic")
    global_path_topic = LaunchConfiguration("global_path_topic")
    local_path_topic = LaunchConfiguration("local_path_topic")
    active_nav_goal_topic = LaunchConfiguration("active_nav_goal_topic")
    object_mission_goal_topic = LaunchConfiguration("object_mission_goal_topic")
    lidar_objects_map_topic = LaunchConfiguration("lidar_objects_map_topic")
    elevated_dynamic_target_topic = LaunchConfiguration("elevated_dynamic_target_topic")
    current_waypoint_topic = LaunchConfiguration("current_waypoint_topic")
    patrol_status_topic = LaunchConfiguration("patrol_status_topic")
    mission_state_topic = LaunchConfiguration("mission_state_topic")
    safety_state_topic = LaunchConfiguration("safety_state_topic")
    height_filter_debug_topic = LaunchConfiguration("height_filter_debug_topic")
    camera_detection_status_topic = LaunchConfiguration("camera_detection_status_topic")
    sound_mission_status_topic = LaunchConfiguration("sound_mission_status_topic")
    gazebo_trial_state_topic = LaunchConfiguration("gazebo_trial_state_topic")
    map_apply_state_topic = LaunchConfiguration("map_apply_state_topic")
    mission_command_topic = LaunchConfiguration("mission_command_topic")
    mapping_command_topic = LaunchConfiguration("mapping_command_topic")
    mode_cmd_topic = LaunchConfiguration("mode_cmd_topic")
    operator_command_topic = LaunchConfiguration("operator_command_topic")
    publish_direct_cmd_vel = LaunchConfiguration("publish_direct_cmd_vel")
    profile = LaunchConfiguration("profile")
    allow_subprocess_launches = LaunchConfiguration("allow_subprocess_launches")
    allow_mapping_launches = LaunchConfiguration("allow_mapping_launches")
    allow_map_save_commands = LaunchConfiguration("allow_map_save_commands")
    allow_localization_launches = LaunchConfiguration("allow_localization_launches")
    demo_script = LaunchConfiguration("demo_script")
    demo_close_on_finish = LaunchConfiguration("demo_close_on_finish")
    default_control_config = PathJoinSubstitution(
        [ugv_tools_share, "config", "waver_4wd_control.yaml"]
    )
    default_waypoint_file = PathJoinSubstitution(
        [ugv_tools_share, "waypoints", "waver_3m_patrol.yaml"]
    )

    # 역할: 실차 기본값에서는 리모콘이 직접 /cmd_vel을 내지 않고 safety mux에 수동 후보만 보낸다.
    remote_panel = Node(
        package="ugv_tools",
        executable="waver_remote_panel",
        name="waver_remote_panel",
        output="screen",
        parameters=[
            control_config_file,
            {
                "cmd_vel_topic": "/cmd_vel",
                "manual_cmd_vel_topic": "/waver/manual_cmd_vel",
                "auto_cmd_vel_topic": "/waver/cmd_vel_nav2",
                "publish_direct_cmd_vel": ParameterValue(publish_direct_cmd_vel, value_type=bool),
                "profile": profile,
                "manual_override_returns_to_auto": True,
                "auto_mode_strategy": auto_mode_strategy,
                "auto_launch_command": auto_launch_command,
                "mapping_launch_command": mapping_launch_command,
                "map_save_command": map_save_command,
                "localization_launch_command": localization_launch_command,
                "lidar_required": require_scan,
                "auto_require_scan": require_scan,
                "min_valid_scan_points": min_valid_scan_points,
                "auto_min_valid_scan_points": min_valid_scan_points,
                "auto_param_file": control_config_file,
                "auto_waypoint_file": waypoint_file,
                "auto_waypoints_csv": auto_waypoints_csv,
                "auto_max_patrol_radius_m": max_patrol_radius_m,
                "map_topic": map_topic,
                "fixed_map_topic": fixed_map_topic,
                "map_display_mode": map_display_mode,
                "amcl_pose_topic": amcl_pose_topic,
                "global_path_topic": global_path_topic,
                "local_path_topic": local_path_topic,
                "active_nav_goal_topic": active_nav_goal_topic,
                "object_mission_goal_topic": object_mission_goal_topic,
                "lidar_objects_map_topic": lidar_objects_map_topic,
                "elevated_dynamic_target_topic": elevated_dynamic_target_topic,
                "current_waypoint_topic": current_waypoint_topic,
                "patrol_status_topic": patrol_status_topic,
                "mission_state_topic": mission_state_topic,
                "safety_state_topic": safety_state_topic,
                "height_filter_debug_topic": height_filter_debug_topic,
                "camera_detection_status_topic": camera_detection_status_topic,
                "sound_mission_status_topic": sound_mission_status_topic,
                "gazebo_trial_state_topic": gazebo_trial_state_topic,
                "map_apply_state_topic": map_apply_state_topic,
                "mission_command_topic": mission_command_topic,
                "mapping_command_topic": mapping_command_topic,
                "mode_cmd_topic": mode_cmd_topic,
                "operator_command_topic": operator_command_topic,
                "allow_subprocess_launches": ParameterValue(
                    allow_subprocess_launches, value_type=bool
                ),
                "allow_mapping_launches": ParameterValue(
                    allow_mapping_launches, value_type=bool
                ),
                "allow_map_save_commands": ParameterValue(
                    allow_map_save_commands, value_type=bool
                ),
                "allow_localization_launches": ParameterValue(
                    allow_localization_launches, value_type=bool
                ),
                "demo_script": demo_script,
                "demo_close_on_finish": ParameterValue(demo_close_on_finish, value_type=bool),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("min_valid_scan_points", default_value="40"),
            DeclareLaunchArgument("control_config_file", default_value=default_control_config),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoint_file),
            DeclareLaunchArgument("max_patrol_radius_m", default_value="3.0"),
            DeclareLaunchArgument("auto_mode_strategy", default_value="mission_nav2"),
            DeclareLaunchArgument("auto_launch_command", default_value=""),
            DeclareLaunchArgument(
                "mapping_launch_command",
                default_value=(
                    "ros2 launch waver_patrol waver_mapping_backend.launch.py "
                    "backend:=gmapping use_sim_time:=true use_rviz:=false "
                    "start_workflow_manager:=false "
                    "start_lidar_bringup:=false "
                    "start_robot_pose_publisher:=false "
                    "scan_topic:=/scan"
                ),
            ),
            DeclareLaunchArgument("map_save_command", default_value=""),
            DeclareLaunchArgument("localization_launch_command", default_value=""),
            DeclareLaunchArgument("map_topic", default_value="/map"),
            DeclareLaunchArgument("fixed_map_topic", default_value="/map_fixed"),
            DeclareLaunchArgument("map_display_mode", default_value="auto"),
            DeclareLaunchArgument("amcl_pose_topic", default_value="/amcl_pose"),
            DeclareLaunchArgument("global_path_topic", default_value="/plan"),
            DeclareLaunchArgument("local_path_topic", default_value="/local_plan"),
            DeclareLaunchArgument("active_nav_goal_topic", default_value="/waver/active_nav_goal"),
            DeclareLaunchArgument("object_mission_goal_topic", default_value="/waver/object_mission_goal"),
            DeclareLaunchArgument("lidar_objects_map_topic", default_value="/waver/lidar_objects_map"),
            DeclareLaunchArgument("elevated_dynamic_target_topic", default_value="/waver/elevated_dynamic_targets"),
            DeclareLaunchArgument("current_waypoint_topic", default_value="/waver/current_waypoint"),
            DeclareLaunchArgument("patrol_status_topic", default_value="/waver/patrol_status"),
            DeclareLaunchArgument("mission_state_topic", default_value="/waver/mission_state"),
            DeclareLaunchArgument("safety_state_topic", default_value="/waver/safety_state"),
            DeclareLaunchArgument("height_filter_debug_topic", default_value="/waver/height_filter_debug"),
            DeclareLaunchArgument("camera_detection_status_topic", default_value="/waver/bird_detector_state"),
            DeclareLaunchArgument("sound_mission_status_topic", default_value="/waver/sound_mission_status"),
            DeclareLaunchArgument("gazebo_trial_state_topic", default_value="/waver/gazebo_trial_state"),
            DeclareLaunchArgument("map_apply_state_topic", default_value="/waver/map_apply_state"),
            DeclareLaunchArgument("mission_command_topic", default_value="/waver/mission_command"),
            DeclareLaunchArgument("mapping_command_topic", default_value="/waver/mapping_command"),
            DeclareLaunchArgument("mode_cmd_topic", default_value="/waver/mode_cmd"),
            DeclareLaunchArgument("operator_command_topic", default_value="/waver/operator_command"),
            DeclareLaunchArgument("profile", default_value="real"),
            DeclareLaunchArgument("publish_direct_cmd_vel", default_value="false"),
            DeclareLaunchArgument("allow_subprocess_launches", default_value="false"),
            DeclareLaunchArgument("allow_mapping_launches", default_value="true"),
            DeclareLaunchArgument("allow_map_save_commands", default_value="true"),
            DeclareLaunchArgument("allow_localization_launches", default_value="false"),
            DeclareLaunchArgument("demo_script", default_value=""),
            DeclareLaunchArgument("demo_close_on_finish", default_value="false"),
            DeclareLaunchArgument(
                "auto_waypoints_csv",
                default_value="",
            ),
            LogInfo(
                msg=(
                    "Waver operator panel: visual remote only. It publishes "
                    "/waver/manual_cmd_vel and /waver/mode_cmd; final /waver/mode and /cmd_vel must come "
                    "from safety_cmd_mux_node on the Jetson backend. Gazebo launch "
                    "commands are blocked from this panel; the SLAM mapping button may "
                    "start only the configured mapping backend."
                )
            ),
            remote_panel,
        ]
    )
