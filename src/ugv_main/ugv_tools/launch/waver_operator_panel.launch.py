# Copyright 2026 Waver project contributors.
"""Operator-PC Waver visual remote panel.

역할:
  - 조작 PC나 Jetson 데스크톱에서 시각화 리모콘만 실행한다.
  - 버튼/키보드는 `/waver/manual_cmd_vel` 후보만 발행하고, AUTO 버튼은 `/waver/mode=AUTO`를 발행한다.
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
                "publish_direct_cmd_vel": False,
                "manual_override_returns_to_auto": True,
                "auto_mode_strategy": auto_mode_strategy,
                "auto_launch_command": auto_launch_command,
                "lidar_required": require_scan,
                "auto_require_scan": require_scan,
                "min_valid_scan_points": min_valid_scan_points,
                "auto_min_valid_scan_points": min_valid_scan_points,
                "auto_param_file": control_config_file,
                "auto_waypoint_file": waypoint_file,
                "auto_waypoints_csv": auto_waypoints_csv,
                "auto_max_patrol_radius_m": max_patrol_radius_m,
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
                "auto_waypoints_csv",
                default_value="",
            ),
            LogInfo(
                msg=(
                    "Waver operator panel: visual remote only. It publishes "
                    "/waver/manual_cmd_vel and /waver/mode; final /cmd_vel must come "
                    "from safety_cmd_mux_node on the Jetson backend."
                )
            ),
            remote_panel,
        ]
    )
