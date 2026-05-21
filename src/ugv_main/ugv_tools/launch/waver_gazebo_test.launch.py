# Copyright 2026 Waver project contributors.
"""Waver Gazebo smoke-test launch that lives inside ugv_tools.

역할:
  - 원본 ugv_gazebo 파일을 수정하지 않고 Gazebo 서버, UGV spawn,
    Waver 노드를 묶는다.
  - 기본은 headless Gazebo + 저속 waypoint patrol이다.
  - 키보드 scripted 테스트는 patrol과 동시에 켜지 않도록 별도 옵션으로 둔다.
  - start_remote_panel:=true이면 GUI 리모콘에서 MANUAL/AUTO를 직접 눌러 실험한다.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 역할: Gazebo 원본 패키지 경로를 substitution으로 참조해 clone 파일 구조를 보존한다.
    ugv_gazebo_share = FindPackageShare("ugv_gazebo")
    ugv_tools_share = FindPackageShare("ugv_tools")

    # 역할: 실험 환경별로 바꿀 수 있는 launch 인자를 선언한다.
    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    start_gazebo = LaunchConfiguration("start_gazebo")
    start_patrol = LaunchConfiguration("start_patrol")
    start_scripted_keyboard = LaunchConfiguration("start_scripted_keyboard")
    start_remote_panel = LaunchConfiguration("start_remote_panel")
    use_minimal_model = LaunchConfiguration("use_minimal_model")
    require_scan = LaunchConfiguration("require_scan")
    min_valid_scan_points = LaunchConfiguration("min_valid_scan_points")
    control_config_file = LaunchConfiguration("control_config_file")
    waypoint_file = LaunchConfiguration("waypoint_file")
    waypoints_csv = LaunchConfiguration("waypoints_csv")
    max_patrol_radius_m = LaunchConfiguration("max_patrol_radius_m")
    loop_count = LaunchConfiguration("loop_count")
    scripted_keys = LaunchConfiguration("scripted_keys")
    remote_demo_script = LaunchConfiguration("remote_demo_script")
    remote_demo_close_on_finish = LaunchConfiguration("remote_demo_close_on_finish")

    default_world = PathJoinSubstitution([ugv_gazebo_share, "worlds", "ugv_world.world"])
    model_file = PathJoinSubstitution([ugv_gazebo_share, "models", model, "model.sdf"])
    minimal_world = PathJoinSubstitution(
        [ugv_tools_share, "worlds", "waver_flat.world"]
    )
    minimal_model_file = PathJoinSubstitution(
        [ugv_tools_share, "models", "waver_safety_sim", "model.sdf"]
    )
    default_control_config = PathJoinSubstitution(
        [ugv_tools_share, "config", "waver_4wd_control.yaml"]
    )
    default_waypoint_file = PathJoinSubstitution(
        [ugv_tools_share, "waypoints", "waver_3m_patrol.yaml"]
    )

    # 역할: start_gazebo와 minimal/original 선택을 함께 반영하는 조건을 만든다.
    original_gazebo_condition = IfCondition(
        PythonExpression(
            ["'", start_gazebo, "' == 'true' and '", use_minimal_model, "' == 'false'"]
        )
    )
    minimal_gazebo_condition = IfCondition(
        PythonExpression(
            ["'", start_gazebo, "' == 'true' and '", use_minimal_model, "' == 'true'"]
        )
    )
    direct_patrol_condition = IfCondition(
        PythonExpression(
            ["'", start_patrol, "' == 'true' and '", start_remote_panel, "' == 'false'"]
        )
    )
    scripted_keyboard_condition = IfCondition(
        PythonExpression(
            [
                "'",
                start_scripted_keyboard,
                "' == 'true' and '",
                start_remote_panel,
                "' == 'false'",
            ]
        )
    )

    # 역할: Gazebo가 로컬 모델을 우선 찾도록 모델 경로를 설정한다.
    gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            PathJoinSubstitution([ugv_gazebo_share, "models"]),
            ":",
            PathJoinSubstitution([ugv_tools_share, "models"]),
            ":",
            PathJoinSubstitution([FindPackageShare("ugv_description"), ".."]),
            ":/usr/share/gazebo-11/models:",
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    # 역할: 인터넷 model database 접근 지연을 막아 Jetson/현장 테스트 시작 시간을 줄인다.
    gazebo_model_database_uri = SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI",
        value="",
    )
    original_model_notice = LogInfo(
        msg=(
            "Waver Gazebo: original ugv_rover is selected. If libros2_livox.so is "
            "missing, use require_scan:=false for drive-only tests or install the "
            "Livox Gazebo plugin before LiDAR safety tests."
        ),
        condition=original_gazebo_condition,
    )
    minimal_model_notice = LogInfo(
        msg=(
            "Waver Gazebo: minimal waver_safety_sim is selected for reliable "
            "remote-panel and /scan safety smoke tests."
        ),
        condition=minimal_gazebo_condition,
    )

    # 역할: 일부 Jetson/SSH 환경에서 Gazebo Classic 렌더 센서가 죽는 일을 줄인다.
    libgl_software = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")
    gazebo_resource_path = SetEnvironmentVariable(
        name="GAZEBO_RESOURCE_PATH",
        value="/usr/share/gazebo-11",
    )

    # 역할: 원본 ugv_gazebo world로 Gazebo를 실행한다.
    gzserver_original = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            world,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
        condition=original_gazebo_condition,
    )

    # 역할: headless smoke test용 최소 world로 Gazebo를 실행한다.
    gzserver_minimal = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            minimal_world,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        output="screen",
        condition=minimal_gazebo_condition,
    )

    # 역할: 데스크톱에서 눈으로 확인할 때만 gzclient를 켠다. SSH/Jetson 기본값은 false다.
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(gui),
    )

    # 역할: Gazebo factory 서비스가 뜬 뒤 원본 ugv_gazebo 모델을 spawn한다.
    spawn_ugv_original = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "gazebo_ros",
                    "spawn_entity.py",
                    "-entity",
                    model,
                    "-file",
                    model_file,
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.05",
                ],
                output="screen",
            )
        ],
        condition=original_gazebo_condition,
    )

    # 역할: 렌더 센서 없는 최소 Waver 시험 모델을 spawn한다.
    spawn_ugv_minimal = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "gazebo_ros",
                    "spawn_entity.py",
                    "-entity",
                    "waver_safety_sim",
                    "-file",
                    minimal_model_file,
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.05",
                ],
                output="screen",
            )
        ],
        condition=minimal_gazebo_condition,
    )

    # 역할: Gazebo /odom과 /scan을 이용해 저속 waypoint patrol 후보를 검증한다.
    patrol_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ugv_tools",
                executable="waver_gazebo_patrol",
                name="waver_gazebo_patrol",
                output="screen",
                parameters=[
                    control_config_file,
                    {
                        "use_sim_time": True,
                        "cmd_vel_topic": "/cmd_vel",
                        "odom_topic": "/odom",
                        "scan_topic": "/scan",
                        "waypoint_file": waypoint_file,
                        "waypoints_csv": waypoints_csv,
                        "max_patrol_radius_m": max_patrol_radius_m,
                        "loop_count": loop_count,
                        "lidar_required": require_scan,
                        "min_valid_scan_points": min_valid_scan_points,
                    },
                ],
            )
        ],
        condition=direct_patrol_condition,
    )

    # 역할: 사람이 없는 테스트에서 keyboard path를 재현한다. patrol과 동시 실행하지 않는 용도다.
    scripted_keyboard_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ugv_tools",
                executable="keyboard_ctrl",
                name="waver_scripted_keyboard",
                output="screen",
                arguments=["--scripted-keys", scripted_keys, "--scripted-interval", "0.10"],
                parameters=[
                    control_config_file,
                    {
                        "use_sim_time": True,
                        "cmd_vel_topic": "/cmd_vel",
                        "enable_scan_assist": True,
                        "lidar_required": require_scan,
                        "min_valid_scan_points": min_valid_scan_points,
                    }
                ],
            )
        ],
        condition=scripted_keyboard_condition,
    )

    # 역할: Gazebo에서 시각화 리모콘을 함께 띄워 방향타/MANUAL/AUTO 버튼을 검증한다.
    remote_panel_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ugv_tools",
                executable="waver_remote_panel",
                name="waver_remote_panel",
                output="screen",
                parameters=[
                    control_config_file,
                    {
                        "use_sim_time": True,
                        "cmd_vel_topic": "/cmd_vel",
                        "manual_cmd_vel_topic": "/waver/manual_cmd_vel",
                        "auto_cmd_vel_topic": "/waver/cmd_vel_auto",
                        "publish_direct_cmd_vel": True,
                        "auto_mode_strategy": "legacy_subprocess",
                        "auto_command": "ros2 run ugv_tools waver_gazebo_patrol",
                        "manual_override_returns_to_auto": False,
                        "odom_topic": "/odom",
                        "scan_topic": "/scan",
                        "lidar_required": require_scan,
                        "auto_require_scan": require_scan,
                        "min_valid_scan_points": min_valid_scan_points,
                        "auto_min_valid_scan_points": min_valid_scan_points,
                        "auto_param_file": control_config_file,
                        "auto_waypoint_file": waypoint_file,
                        "auto_waypoints_csv": waypoints_csv,
                        "auto_max_patrol_radius_m": max_patrol_radius_m,
                        "auto_use_sim_time": True,
                        "demo_script": remote_demo_script,
                        "demo_close_on_finish": remote_demo_close_on_finish,
                    },
                ],
            )
        ],
        condition=IfCondition(start_remote_panel),
    )

    # 역할: ros2 launch에서 사용자가 바로 볼 수 있는 기본값을 한곳에 모은다.
    launch_arguments = [
        DeclareLaunchArgument("model", default_value="ugv_rover"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("start_gazebo", default_value="true"),
        DeclareLaunchArgument("start_patrol", default_value="true"),
        DeclareLaunchArgument("start_scripted_keyboard", default_value="false"),
        DeclareLaunchArgument("start_remote_panel", default_value="false"),
        DeclareLaunchArgument("use_minimal_model", default_value="false"),
        DeclareLaunchArgument("require_scan", default_value="false"),
        DeclareLaunchArgument("min_valid_scan_points", default_value="0"),
        DeclareLaunchArgument("control_config_file", default_value=default_control_config),
        DeclareLaunchArgument("waypoint_file", default_value=default_waypoint_file),
        DeclareLaunchArgument(
            "waypoints_csv",
            default_value="",
        ),
        DeclareLaunchArgument("max_patrol_radius_m", default_value="3.0"),
        DeclareLaunchArgument("loop_count", default_value="1"),
        DeclareLaunchArgument("scripted_keys", default_value="wwaaddk"),
        DeclareLaunchArgument("remote_demo_script", default_value=""),
        DeclareLaunchArgument("remote_demo_close_on_finish", default_value="false"),
    ]

    return LaunchDescription(
        [
            *launch_arguments,
            gazebo_model_database_uri,
            original_model_notice,
            minimal_model_notice,
            libgl_software,
            gazebo_resource_path,
            gazebo_model_path,
            gzserver_original,
            gzserver_minimal,
            gzclient,
            spawn_ugv_original,
            spawn_ugv_minimal,
            patrol_node,
            scripted_keyboard_node,
            remote_panel_node,
        ]
    )
