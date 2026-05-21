# Copyright 2026 Waver project contributors.
"""Real-robot Waver control launch inside ugv_tools.

역할:
  - 기존 Waveshare 패키지는 건드리지 않고 Waver 실차 제어 경로만 묶는다.
  - legacy one-machine 수동/단순 순찰 확인용으로 GUI 리모콘이 직접 /cmd_vel을 낸다.
  - serial bridge 하나만 /cmd_vel을 WAVE ROVER JSON으로 변환한다.

주의:
  - Nav2/mission 통합 실증은 `waver_patrol waver_nav2_radar_bird_mission.launch.py`
    backend와 `waver_operator_panel.launch.py` 조합을 사용한다.
  - 기존 ugv_driver/app.py가 같은 serial 포트를 잡고 있으면 start_serial_bridge:=true 금지.
  - LiDAR가 준비되지 않은 상태에서 실차를 움직이면 안 되므로 require_scan 기본은 true다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 역할: 실차 환경에서 자주 바꾸는 값만 launch 인자로 노출한다.
    ugv_tools_share = FindPackageShare("ugv_tools")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    reconnect_interval = LaunchConfiguration("reconnect_interval")
    start_serial_bridge = LaunchConfiguration("start_serial_bridge")
    start_remote_panel = LaunchConfiguration("start_remote_panel")
    require_scan = LaunchConfiguration("require_scan")
    min_valid_scan_points = LaunchConfiguration("min_valid_scan_points")
    control_config_file = LaunchConfiguration("control_config_file")
    waypoint_file = LaunchConfiguration("waypoint_file")
    auto_waypoints_csv = LaunchConfiguration("auto_waypoints_csv")
    max_patrol_radius_m = LaunchConfiguration("max_patrol_radius_m")
    default_control_config = PathJoinSubstitution(
        [ugv_tools_share, "config", "waver_4wd_control.yaml"]
    )
    default_waypoint_file = PathJoinSubstitution(
        [ugv_tools_share, "waypoints", "waver_3m_patrol.yaml"]
    )

    # 역할: /cmd_vel을 실제 WAVE ROVER ESP32 JSON 명령으로 바꾸는 유일한 하드웨어 경로다.
    serial_bridge = Node(
        package="ugv_tools",
        executable="waver_cmd_vel_serial_bridge",
        name="waver_cmd_vel_serial_bridge",
        output="screen",
        arguments=[
            "--serial-port",
            serial_port,
            "--baudrate",
            baudrate,
            "--reconnect-interval",
            reconnect_interval,
        ],
        parameters=[
            control_config_file,
            {
                "cmd_vel_topic": "/cmd_vel",
                "lidar_required": require_scan,
                "min_valid_scan_points": min_valid_scan_points,
            },
        ],
        condition=IfCondition(start_serial_bridge),
    )

    # 역할: legacy 한 대 테스트에서는 안전 mux가 없으므로 직접 /cmd_vel을 발행한다.
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
                "auto_cmd_vel_topic": "/waver/cmd_vel_auto",
                "publish_direct_cmd_vel": True,
                "auto_mode_strategy": "legacy_subprocess",
                "auto_command": "ros2 run ugv_tools waver_gazebo_patrol",
                "manual_override_returns_to_auto": False,
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
        condition=IfCondition(start_remote_panel),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyTHS0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("reconnect_interval", default_value="1.0"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("start_remote_panel", default_value="true"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("min_valid_scan_points", default_value="40"),
            DeclareLaunchArgument("control_config_file", default_value=default_control_config),
            DeclareLaunchArgument("waypoint_file", default_value=default_waypoint_file),
            DeclareLaunchArgument("max_patrol_radius_m", default_value="3.0"),
            DeclareLaunchArgument(
                "auto_waypoints_csv",
                default_value="",
            ),
            LogInfo(
                msg=(
                    "Waver real control: start_serial_bridge is false by default. "
                    "Set true only after stopping ugv_driver/app.py on the same serial port."
                )
            ),
            serial_bridge,
            remote_panel,
        ]
    )
