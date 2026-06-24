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
from launch_ros.parameter_descriptions import ParameterValue
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
    start_bird_manager = LaunchConfiguration("start_bird_manager")
    start_data_logger = LaunchConfiguration("start_data_logger")
    start_scripted_keyboard = LaunchConfiguration("start_scripted_keyboard")
    start_remote_panel = LaunchConfiguration("start_remote_panel")
    use_minimal_model = LaunchConfiguration("use_minimal_model")
    spawn_ugv = LaunchConfiguration("spawn_ugv")
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
    remote_publish_direct_cmd_vel = LaunchConfiguration("remote_publish_direct_cmd_vel")
    data_log_dir = LaunchConfiguration("data_log_dir")
    data_session_name = LaunchConfiguration("data_session_name")
    start_lidar_perception = LaunchConfiguration("start_lidar_perception")
    publish_bird_manager_targets = LaunchConfiguration("publish_bird_manager_targets")
    bird_manager_active_birds = LaunchConfiguration("bird_manager_active_birds")
    bird_manager_z_min_m = LaunchConfiguration("bird_manager_z_min_m")
    bird_manager_z_max_m = LaunchConfiguration("bird_manager_z_max_m")
    bird_manager_min_speed_mps = LaunchConfiguration("bird_manager_min_speed_mps")
    bird_manager_max_speed_mps = LaunchConfiguration("bird_manager_max_speed_mps")
    bird_manager_min_xy_radius_m = LaunchConfiguration("bird_manager_min_xy_radius_m")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    pointcloud_target_frame = LaunchConfiguration("pointcloud_target_frame")
    lidar_object_min_height_m = LaunchConfiguration("lidar_object_min_height_m")
    lidar_object_max_height_m = LaunchConfiguration("lidar_object_max_height_m")
    elevated_target_min_height_m = LaunchConfiguration("elevated_target_min_height_m")
    start_lidar_target_follow = LaunchConfiguration("start_lidar_target_follow")
    lidar_follow_enable_cmd_vel = LaunchConfiguration("lidar_follow_enable_cmd_vel")
    lidar_follow_warning_distance_m = LaunchConfiguration("lidar_follow_warning_distance_m")
    lidar_follow_start_distance_m = LaunchConfiguration("lidar_follow_start_distance_m")
    lidar_follow_hold_distance_m = LaunchConfiguration("lidar_follow_hold_distance_m")
    lidar_follow_max_track_distance_m = LaunchConfiguration("lidar_follow_max_track_distance_m")

    default_world = PathJoinSubstitution([ugv_gazebo_share, "worlds", "ugv_world.world"])
    model_file = PathJoinSubstitution([ugv_gazebo_share, "models", model, "model.sdf"])
    robot_urdf = PathJoinSubstitution(
        [
            ugv_gazebo_share,
            "urdf",
            PythonExpression(["'", model, "' + '.urdf'"]),
        ]
    )
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
        [ugv_tools_share, "waypoints", "waver_10m_patrol.yaml"]
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
    spawn_original_condition = IfCondition(
        PythonExpression(
            [
                "'",
                start_gazebo,
                "' == 'true' and '",
                use_minimal_model,
                "' == 'false' and '",
                spawn_ugv,
                "' == 'true'",
            ]
        )
    )
    spawn_minimal_condition = IfCondition(
        PythonExpression(
            [
                "'",
                start_gazebo,
                "' == 'true' and '",
                use_minimal_model,
                "' == 'true' and '",
                spawn_ugv,
                "' == 'true'",
            ]
        )
    )
    original_model_condition = IfCondition(
        PythonExpression(
            ["'", use_minimal_model, "' == 'false'"]
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
        condition=spawn_original_condition,
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
        condition=spawn_minimal_condition,
    )

    # 역할: livox -> base_link 및 base_link -> base_footprint 정적 TF를 제공해
    # PointCloud2 객체 후보를 Waver 공통 좌표계로 변환할 수 있게 한다.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        arguments=[robot_urdf],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=original_model_condition,
    )

    # 역할: pcd_cluster_pkg가 Gazebo Livox PointCloud2를 DBSCAN 클러스터링하고
    # 실제 LiDAR 객체 후보 /waver/lidar_objects를 만든다.
    # bird_manager는 모델 이동만 담당하고 Waver target 토픽은 이 체인이 만든다.
    pcd_cluster_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="pcd_cluster_pkg",
                executable="cluster_node",
                name="pcd_cluster_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "pointcloud_topic": pointcloud_topic,
                        "odom_topic": "/odom",
                        "lidar_objects_topic": "/waver/lidar_objects",
                        "state_topic": "/waver/lidar_objects_state",
                        "marker_topic": "/cluster_markers",
                        "filtered_points_topic": "/filtered_points",
                        "target_frame": "odom",
                        "ground_z_limit": 2.0,
                        "roi_min_range": 0.2,
                        "roi_max_range": 20.0,
                        "dbscan_eps": 0.75,
                        "dbscan_min_samples": 3,
                        "min_cluster_points": 3,
                        "trackable_max_size_x": 2.5,
                        "trackable_max_size_y": 2.5,
                        "trackable_max_size_z": 2.0,
                        "trackable_min_centroid_z": 2.0,
                        "max_input_points": 12000,
                        "enable_cmd_vel_output": False,
                    }
                ],
            )
        ],
        condition=IfCondition(start_lidar_perception),
    )

    moving_object_map_transform_node = TimerAction(
        period=7.5,
        actions=[
            Node(
                package="waver_patrol",
                executable="moving_object_map_transform_node",
                name="moving_object_map_transform_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/waver/lidar_objects",
                        "input_type": "pose_array",
                        "output_pose_array_topic": "/waver/lidar_objects_map",
                        "debug_marker_topic": "/waver/moving_objects_map_marker",
                        "target_frame": "odom",
                        "fallback_frame": "odom",
                        "base_frame": "base_link",
                        "use_latest_tf": True,
                    }
                ],
            )
        ],
        condition=IfCondition(start_lidar_perception),
    )

    # 역할: LiDAR가 계속 갱신하는 최종 동적 타깃 좌표를 받아
    # 거리별 경고/목표 발행/차체 방향 추적 명령을 수행한다.
    lidar_target_follow_node = TimerAction(
        period=8.5,
        actions=[
            Node(
                package="pcd_cluster_pkg",
                executable="lidar_target_follow_node",
                name="lidar_target_follow_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "target_topic": "/waver/elevated_dynamic_targets",
                        "odom_topic": "/odom",
                        "cmd_vel_topic": "/cmd_vel",
                        "goal_topic": "/waver/object_mission_goal",
                        "state_topic": "/waver/lidar_target_follow_state",
                        "sound_status_topic": "/waver/sound_mission_status",
                        "sound_request_topic": "/waver/sound_alert_request",
                        "enable_cmd_vel_output": ParameterValue(
                            lidar_follow_enable_cmd_vel,
                            value_type=bool,
                        ),
                        "min_target_height_m": ParameterValue(elevated_target_min_height_m, value_type=float),
                        "max_target_height_m": ParameterValue(lidar_object_max_height_m, value_type=float),
                        "warning_distance_m": ParameterValue(lidar_follow_warning_distance_m, value_type=float),
                        "follow_start_distance_m": ParameterValue(lidar_follow_start_distance_m, value_type=float),
                        "hold_distance_m": ParameterValue(lidar_follow_hold_distance_m, value_type=float),
                        "goal_standoff_m": ParameterValue(lidar_follow_hold_distance_m, value_type=float),
                        "max_track_distance_m": ParameterValue(lidar_follow_max_track_distance_m, value_type=float),
                    }
                ],
            )
        ],
        condition=IfCondition(start_lidar_target_follow),
    )

    moving_object_motion_filter_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="waver_patrol",
                executable="moving_object_motion_filter_node",
                name="moving_object_motion_filter_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/waver/lidar_objects_map",
                        "raw_input_topic": "/waver/lidar_objects",
                        "target_min_height_m": ParameterValue(elevated_target_min_height_m, value_type=float),
                        "target_max_height_m": 30.0,
                        "z_source_mode": "pointcloud",
                        "require_3d_z_source": True,
                        "min_dynamic_motion_m": 0.2,
                        "min_dynamic_velocity_mps": 0.05,
                        "min_tracking_duration_sec": 0.6,
                        "require_consecutive_dynamic_frames": 2,
                        "track_match_gate_m": 3.0,
                        "max_sample_step_m": 4.0,
                        "elevated_targets_topic": "/waver/elevated_dynamic_targets",
                        "height_debug_topic": "/waver/height_filter_debug",
                        "publish_detection_classification": True,
                        "detected_target_class": "bird",
                        "detected_target_confidence": 0.80,
                    }
                ],
            )
        ],
        condition=IfCondition(start_lidar_perception),
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

    # 역할: 15 m Gazebo map의 bird를 3 m 임계값 바로 위에서 아주 느리게 움직이고 target 토픽을 발행한다.
    bird_manager_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="ugv_gazebo",
                executable="bird_manager.py",
                name="bird_manager",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "active_birds": bird_manager_active_birds,
                        "publish_waver_detection_topics": ParameterValue(
                            publish_bird_manager_targets,
                            value_type=bool,
                        ),
                        "z_min_m": ParameterValue(bird_manager_z_min_m, value_type=float),
                        "z_max_m": ParameterValue(bird_manager_z_max_m, value_type=float),
                        "min_speed_mps": ParameterValue(bird_manager_min_speed_mps, value_type=float),
                        "max_speed_mps": ParameterValue(bird_manager_max_speed_mps, value_type=float),
                        "min_xy_radius_m": ParameterValue(bird_manager_min_xy_radius_m, value_type=float),
                    }
                ],
            )
        ],
        condition=IfCondition(start_bird_manager),
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
                        "auto_cmd_vel_topic": "/cmd_vel",
                        "profile": "gazebo",
                        "publish_direct_cmd_vel": ParameterValue(
                            remote_publish_direct_cmd_vel,
                            value_type=bool,
                        ),
                        "allow_subprocess_launches": True,
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

    # 역할: 논문/실험용 CSV와 summary JSON을 1초 간격으로 저장한다.
    data_logger_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="ugv_gazebo",
                executable="gazebo_trial_data_logger.py",
                name="gazebo_trial_data_logger",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "output_dir": data_log_dir,
                        "session_name": data_session_name,
                        "sample_period_s": 1.0,
                        "map_side_m": 15.0,
                        "patrol_side_m": 10.0,
                    }
                ],
            )
        ],
        condition=IfCondition(start_data_logger),
    )

    # 역할: ros2 launch에서 사용자가 바로 볼 수 있는 기본값을 한곳에 모은다.
    launch_arguments = [
        DeclareLaunchArgument("model", default_value="ugv_rover"),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("start_gazebo", default_value="true"),
        DeclareLaunchArgument("start_patrol", default_value="true"),
        DeclareLaunchArgument("start_bird_manager", default_value="true"),
        DeclareLaunchArgument("start_data_logger", default_value="true"),
        DeclareLaunchArgument("start_scripted_keyboard", default_value="false"),
        DeclareLaunchArgument("start_remote_panel", default_value="false"),
        DeclareLaunchArgument("start_lidar_perception", default_value="true"),
        DeclareLaunchArgument("publish_bird_manager_targets", default_value="false"),
        DeclareLaunchArgument("bird_manager_active_birds", default_value="bird_test_target"),
        DeclareLaunchArgument("bird_manager_z_min_m", default_value="3.1"),
        DeclareLaunchArgument("bird_manager_z_max_m", default_value="3.4"),
        DeclareLaunchArgument("bird_manager_min_speed_mps", default_value="0.05"),
        DeclareLaunchArgument("bird_manager_max_speed_mps", default_value="0.18"),
        DeclareLaunchArgument("bird_manager_min_xy_radius_m", default_value="3.5"),
        DeclareLaunchArgument("pointcloud_topic", default_value="/mid360_PointCloud2"),
        DeclareLaunchArgument("pointcloud_target_frame", default_value="base_link"),
        DeclareLaunchArgument("lidar_object_min_height_m", default_value="3.0"),
        DeclareLaunchArgument("lidar_object_max_height_m", default_value="8.0"),
        DeclareLaunchArgument("elevated_target_min_height_m", default_value="3.0"),
        DeclareLaunchArgument("start_lidar_target_follow", default_value="false"),
        DeclareLaunchArgument("lidar_follow_enable_cmd_vel", default_value="false"),
        DeclareLaunchArgument("lidar_follow_warning_distance_m", default_value="3.0"),
        DeclareLaunchArgument("lidar_follow_start_distance_m", default_value="3.0"),
        DeclareLaunchArgument("lidar_follow_hold_distance_m", default_value="1.8"),
        DeclareLaunchArgument("lidar_follow_max_track_distance_m", default_value="15.0"),
        DeclareLaunchArgument("use_minimal_model", default_value="false"),
        DeclareLaunchArgument("spawn_ugv", default_value="true"),
        DeclareLaunchArgument("require_scan", default_value="false"),
        DeclareLaunchArgument("min_valid_scan_points", default_value="0"),
        DeclareLaunchArgument("control_config_file", default_value=default_control_config),
        DeclareLaunchArgument("waypoint_file", default_value=default_waypoint_file),
        DeclareLaunchArgument(
            "waypoints_csv",
            default_value="",
        ),
        DeclareLaunchArgument("max_patrol_radius_m", default_value="7.2"),
        DeclareLaunchArgument("loop_count", default_value="1"),
        DeclareLaunchArgument("scripted_keys", default_value="wwaaddk"),
        DeclareLaunchArgument("remote_demo_script", default_value=""),
        DeclareLaunchArgument("remote_demo_close_on_finish", default_value="false"),
        DeclareLaunchArgument("remote_publish_direct_cmd_vel", default_value="true"),
        DeclareLaunchArgument("data_log_dir", default_value="~/ugv_ws/bird_patrol_data"),
        DeclareLaunchArgument("data_session_name", default_value="bird_patrol_10m"),
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
            robot_state_publisher_node,
            spawn_ugv_original,
            spawn_ugv_minimal,
            bird_manager_node,
            pcd_cluster_node,
            moving_object_map_transform_node,
            moving_object_motion_filter_node,
            lidar_target_follow_node,
            patrol_node,
            data_logger_node,
            scripted_keyboard_node,
            remote_panel_node,
        ]
    )
