# Copyright 2026 Waver project contributors.
"""Jetson-side Waver robot backend.

역할:
  - Jetson에서 실차 하드웨어와 직접 맞닿는 노드만 실행한다.
  - GUI 리모콘/RViz/Foxglove 같은 조작 화면은 보통 조작 PC에서 실행한다.
  - start_serial_bridge:=true일 때만 /cmd_vel을 WAVE ROVER serial JSON으로 보낸다.

주의:
  - 기존 ugv_driver/app.py가 같은 serial port를 사용 중이면 start_serial_bridge:=true 금지.
  - 이 launch는 화면을 띄우지 않는다. 실외 운용 시 Jetson은 headless backend로 두는 구성이 안정적이다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 역할: Jetson에서 바뀔 수 있는 serial/safety 값을 launch 인자로 노출한다.
    ugv_tools_share = FindPackageShare("ugv_tools")
    serial_port = LaunchConfiguration("serial_port")
    baudrate = LaunchConfiguration("baudrate")
    reconnect_interval = LaunchConfiguration("reconnect_interval")
    start_serial_bridge = LaunchConfiguration("start_serial_bridge")
    use_existing_ugv_bringup = LaunchConfiguration("use_existing_ugv_bringup")
    existing_ugv_serial_port = LaunchConfiguration("existing_ugv_serial_port")
    require_scan = LaunchConfiguration("require_scan")
    min_valid_scan_points = LaunchConfiguration("min_valid_scan_points")
    control_config_file = LaunchConfiguration("control_config_file")
    default_control_config = PathJoinSubstitution(
        [ugv_tools_share, "config", "waver_4wd_control.yaml"]
    )
    existing_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("ugv_bringup"), "launch", "bringup_lidar.launch.py"]
    )
    serial_bridge_condition = IfCondition(
        PythonExpression(
            [
                "'",
                start_serial_bridge,
                "' == 'true' and '",
                use_existing_ugv_bringup,
                "' == 'false'",
            ]
        )
    )

    # 역할: 기존 ugv_bringup/ugv_driver가 사용할 serial 포트를 환경변수로 맞춘다.
    existing_serial_env = SetEnvironmentVariable(
        name="SERIAL_PORT",
        value=existing_ugv_serial_port,
    )

    # 역할: 원본 Waveshare/FSD bringup을 그대로 띄워 센서, odom, ugv_driver를 사용한다.
    existing_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(existing_bringup_launch),
        launch_arguments={
            "use_rviz": "false",
            "pub_odom_tf": "true",
        }.items(),
        condition=IfCondition(use_existing_ugv_bringup),
    )

    # 역할: 조작 PC나 자율 노드가 낸 최종 /cmd_vel을 WAVE ROVER JSON으로 변환하는 하드웨어 경로다.
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
        condition=serial_bridge_condition,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyTHS0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("reconnect_interval", default_value="1.0"),
            DeclareLaunchArgument("start_serial_bridge", default_value="false"),
            DeclareLaunchArgument("use_existing_ugv_bringup", default_value="false"),
            DeclareLaunchArgument("existing_ugv_serial_port", default_value="/dev/ttyTHS1"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("min_valid_scan_points", default_value="40"),
            DeclareLaunchArgument("control_config_file", default_value=default_control_config),
            LogInfo(
                msg=(
                    "Waver robot backend: GUI is not started here. Run "
                    "waver_operator_panel.launch.py on the operator PC, or use "
                    "waver_real_control.launch.py for one-machine testing."
                )
            ),
            LogInfo(
                msg=(
                    "Two hardware command modes exist: use_existing_ugv_bringup:=true "
                    "uses original ugv_driver, start_serial_bridge:=true uses Waver "
                    "serial bridge. Do not run both on the same serial port."
                )
            ),
            existing_serial_env,
            existing_bringup,
            serial_bridge,
        ]
    )
