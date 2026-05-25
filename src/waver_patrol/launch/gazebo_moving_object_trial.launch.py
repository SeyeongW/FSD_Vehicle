from __future__ import annotations

import os
import glob

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    ugv_tools_share = get_package_share_directory("ugv_tools")
    try:
        ugv_description_model_path = os.path.dirname(get_package_share_directory("ugv_description"))
    except PackageNotFoundError:
        ugv_description_model_path = ""
    mission_launch = os.path.join(waver_share, "launch", "waver_nav2_radar_bird_mission.launch.py")
    trial_waypoints = os.path.join(waver_share, "waypoints", "gazebo_moving_object_trial.yaml")
    default_world = os.path.join(ugv_share, "worlds", "ugv_world.world")
    rover_model_sdf = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")
    minimal_model_sdf = os.path.join(ugv_tools_share, "models", "waver_safety_sim", "model.sdf")
    default_map_yaml = os.path.join(ugv_share, "maps", "map.yaml")
    bird_sdf = os.path.join(ugv_share, "models", "bird", "model.sdf")
    bird_manager_py = os.path.normpath(os.path.join(ugv_share, "..", "..", "lib", "ugv_gazebo", "bird_manager.py"))
    if not os.path.isfile(bird_manager_py):
        matches = glob.glob(os.path.expanduser("~/ros2_ws/src/**/bird_manager.py"), recursive=True)
        if matches:
            bird_manager_py = matches[0]

    trial_id = LaunchConfiguration("trial_id")
    min_dynamic_motion = LaunchConfiguration("min_dynamic_motion_m")
    min_dynamic_velocity = LaunchConfiguration("min_dynamic_velocity_mps")
    target_min_height = LaunchConfiguration("target_min_height_m")
    use_gui = LaunchConfiguration("use_gui")
    record_bag = LaunchConfiguration("record_bag")
    output_root = LaunchConfiguration("output_root")
    world_file = LaunchConfiguration("world_file")
    start_gazebo = LaunchConfiguration("start_gazebo")
    default_mode = LaunchConfiguration("default_mode")
    robot_spawn_x = LaunchConfiguration("robot_spawn_x")
    robot_spawn_y = LaunchConfiguration("robot_spawn_y")
    robot_spawn_z = LaunchConfiguration("robot_spawn_z")
    robot_spawn_yaw = LaunchConfiguration("robot_spawn_yaw")
    use_gui_and_start_gazebo = PythonExpression(["'", start_gazebo, "' == 'true' and '", use_gui, "' == 'true'"])

    rosbag_command_text = PythonExpression(
        [
            "'mkdir -p ~/ros2_ws/FSD_Vehicle/experiments_result/gazebo_trial_",
            trial_id,
            "/rosbag; ros2 bag record "
            "/tf /tf_static /clock /scan /odom /cmd_vel /waver/cmd_vel_nav2 "
            "/waver/lidar_objects /waver/lidar_objects_map /waver/moving_objects_map_marker "
            "/waver/moving_object_track /waver/moving_target_valid /waver/mission_state "
            "/waver/mission_event /waver/current_waypoint /waver/active_nav_goal "
            "/waver/object_mission_goal /waver/object_mission_goal_active "
            "/waver/camera_detection_state /waver/target_class /waver/target_confidence "
            "/waver/bird_confirmed /waver/sound_alert_state /waver/sound_task_done "
            "/gazebo/model_states "
            "-o ~/ros2_ws/FSD_Vehicle/experiments_result/gazebo_trial_",
            trial_id,
            "/rosbag/trial_",
            trial_id,
            "_$(date +%Y%m%d_%H%M%S)'",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("trial_id", default_value="1"),
            DeclareLaunchArgument(
                "target_scenario_id",
                default_value="0",
                description=(
                    "Gazebo target trajectory scenario. 0 follows trial_id for H1/H2/H3 tests; "
                    "1 forces the elevated dynamic patrol-interrupt scenario."
                ),
            ),
            DeclareLaunchArgument(
                "target_min_height_m",
                default_value="3.0",
                description="Height threshold for elevated dynamic targets. This is not a motion-distance threshold.",
            ),
            DeclareLaunchArgument(
                "min_dynamic_motion_m",
                default_value="0.2",
                description="Small compensated map/odom motion threshold used only for dynamic/static classification.",
            ),
            DeclareLaunchArgument("min_dynamic_velocity_mps", default_value="0.05"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("start_gazebo", default_value="true"),
            DeclareLaunchArgument(
                "default_mode",
                default_value="STANDBY",
                description="Initial mission/safety mode. Use AUTO only for scripted validation; STANDBY keeps rover still until remote panel changes mode.",
            ),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            DeclareLaunchArgument("spawn_target", default_value="false"),
            DeclareLaunchArgument("spawn_ugv_bird_single", default_value="false"),
            DeclareLaunchArgument("spawn_ugv_bird_swarm", default_value="false"),
            DeclareLaunchArgument("target_sdf_file", default_value=bird_sdf),
            DeclareLaunchArgument("enable_ugv_bird_manager", default_value="false"),
            DeclareLaunchArgument("enable_gazebo_bird_bridge", default_value="false"),
            DeclareLaunchArgument("enable_fake_camera_classification", default_value="false"),
            DeclareLaunchArgument("enable_fake_sound", default_value="false"),
            DeclareLaunchArgument("robot_entity", default_value="ugv_rover"),
            DeclareLaunchArgument("robot_spawn_x", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_z", default_value="0.15"),
            DeclareLaunchArgument("robot_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "robot_sdf_file",
                default_value=rover_model_sdf,
                description=(
                    "Gazebo robot SDF. Defaults to the existing ugv_gazebo/models/ugv_rover/model.sdf. "
                    "Use ugv_tools/models/waver_safety_sim/model.sdf only for lightweight CI fallback."
                ),
            ),
            DeclareLaunchArgument("enable_gazebo_map_path_visualizer", default_value="true"),
            DeclareLaunchArgument(
                "publish_static_map_to_odom_tf",
                default_value="true",
                description=(
                    "Gazebo-only fallback TF for airport trials without AMCL/SLAM. "
                    "Set false when a localization backend publishes map->odom."
                ),
            ),
            DeclareLaunchArgument(
                "publish_static_rover_sensor_tf",
                default_value="true",
                description=(
                    "Gazebo-only fixed TF fallback for ugv_rover links when no robot_state_publisher is running."
                ),
            ),
            DeclareLaunchArgument("dynamic_obstacle_topic", default_value="/waver/dynamic_obstacle_map"),
            DeclareLaunchArgument("enable_dynamic_obstacle_detour", default_value="false"),
            DeclareLaunchArgument("enable_simple_nav2_avoidance", default_value="false"),
            DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
            DeclareLaunchArgument("enable_mission_stack", default_value="false"),
            DeclareLaunchArgument("enable_simple_nav2_cmd_sim", default_value="false"),
            DeclareLaunchArgument("enable_moving_object_motion_filter", default_value="false"),
            DeclareLaunchArgument("use_nav2", default_value="false"),
            DeclareLaunchArgument("require_scan", default_value="false"),
            DeclareLaunchArgument("enable_cluster_node", default_value="false"),
            DeclareLaunchArgument("enable_experiment_logger", default_value="true"),
            DeclareLaunchArgument("enable_trial_logger", default_value="false"),
            DeclareLaunchArgument("record_bag", default_value="false"),
            DeclareLaunchArgument("output_root", default_value="~/ros2_ws/FSD_Vehicle/experiments_result"),
            DeclareLaunchArgument(
                "world_file",
                default_value=default_world,
                description=(
                    "Gazebo world file. Defaults to the existing ugv_gazebo/worlds/ugv_world.world "
                    "so Waver trials run in the repository's airport patrol map."
                ),
            ),
            DeclareLaunchArgument("waypoint_file", default_value=trial_waypoints),
            DeclareLaunchArgument("trial_duration_sec", default_value="70"),
            DeclareLaunchArgument("target_motion_duration_sec", default_value="8.0"),
            DeclareLaunchArgument("target_z", default_value="3.2"),
            DeclareLaunchArgument("target_gazebo_move_delay_sec", default_value="8.0"),
            DeclareLaunchArgument(
                "target_start_on_mission_command",
                default_value="false",
                description=(
                    "When true, the Gazebo elevated target waits for START_PATROL/AUTO_MODE "
                    "before publishing its moving cluster trajectory."
                ),
            ),
            DeclareLaunchArgument("gazebo_sim_max_linear_speed", default_value="1.0"),
            DeclareLaunchArgument(
                "gazebo_goal_tolerance_m",
                default_value="0.85",
                description="Gazebo-only simple-nav tolerance. Real robot Nav2 tolerances are configured separately.",
            ),
            DeclareLaunchArgument("post_target_resume_cooldown_sec", default_value="12.0"),
            SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI", value=""),
            SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
            SetEnvironmentVariable(name="GAZEBO_RESOURCE_PATH", value="/usr/share/gazebo-11"),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=(
                    f"{os.path.join(ugv_share, 'models')}:"
                    f"{os.path.join(ugv_tools_share, 'models')}:"
                    f"{ugv_description_model_path}:"
                    f"/usr/share/gazebo-11/models:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
                ),
            ),
            LogInfo(
                msg=(
                    "Gazebo moving-object trial: fake cluster publisher uses /waver/lidar_objects; "
                    "cluster/mission nodes never publish final /cmd_vel."
                )
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gazebo_static_map_to_odom_tf",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_static_map_to_odom_tf")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gazebo_static_base_footprint_to_base_link_tf",
                arguments=["0.00046", "0", "0.08", "0", "0", "0", "base_footprint", "base_link"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_static_rover_sensor_tf")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gazebo_static_base_footprint_to_imu_tf",
                arguments=["0.00046", "0", "0.08", "0", "0", "0", "base_footprint", "base_imu_link"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_static_rover_sensor_tf")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gazebo_static_base_link_to_livox_tf",
                arguments=["-0.05", "0", "0.08", "0", "0", "0", "base_link", "livox"],
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_static_rover_sensor_tf")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gazebo_static_base_footprint_to_3d_camera_tf",
                arguments=[
                    "0.06577",
                    "0",
                    "0.101953",
                    "0",
                    "0",
                    "0",
                    "base_footprint",
                    "3d_camera_link",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("publish_static_rover_sensor_tf")),
            ),
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    "--verbose",
                    world_file,
                    "-s",
                    "libgazebo_ros_init.so",
                    "-s",
                    "libgazebo_ros_factory.so",
                ],
                output="screen",
                condition=IfCondition(start_gazebo),
            ),
            ExecuteProcess(cmd=["gzclient"], output="screen", condition=IfCondition(use_gui_and_start_gazebo)),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                            "-entity",
                            LaunchConfiguration("robot_entity"),
                            "-file",
                            LaunchConfiguration("robot_sdf_file"),
                            "-x",
                            robot_spawn_x,
                            "-y",
                            robot_spawn_y,
                            "-z",
                            robot_spawn_z,
                            "-Y",
                            robot_spawn_yaw,
                        ],
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("spawn_robot")),
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                            "-entity",
                            "bird_test_target",
                            "-file",
                            LaunchConfiguration("target_sdf_file"),
                            "-x",
                            "2.0",
                            "-y",
                            "0.0",
                            "-z",
                            LaunchConfiguration("target_z"),
                        ],
                        output="screen",
                        condition=IfCondition(LaunchConfiguration("spawn_target")),
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=[
                            "-entity",
                            "bird_single",
                            "-file",
                            bird_sdf,
                            "-x",
                            "3.0",
                            "-y",
                            "2.0",
                            "-z",
                            "6.0",
                        ],
                        output="screen",
                    )
                ],
                condition=IfCondition(LaunchConfiguration("spawn_ugv_bird_single")),
            ),
            TimerAction(
                period=4.8,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=["-entity", "bird_swarm_1", "-file", bird_sdf, "-x", "-6.0", "-y", "0.0", "-z", "6.5"],
                        output="screen",
                    ),
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=["-entity", "bird_swarm_2", "-file", bird_sdf, "-x", "6.0", "-y", "0.0", "-z", "6.5"],
                        output="screen",
                    ),
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=["-entity", "bird_swarm_3", "-file", bird_sdf, "-x", "-3.0", "-y", "-5.5", "-z", "7.0"],
                        output="screen",
                    ),
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=["-entity", "bird_swarm_4", "-file", bird_sdf, "-x", "3.0", "-y", "-5.5", "-z", "5.8"],
                        output="screen",
                    ),
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=["-entity", "bird_swarm_5", "-file", bird_sdf, "-x", "0.0", "-y", "6.0", "-z", "6.8"],
                        output="screen",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("spawn_ugv_bird_swarm")),
            ),
            TimerAction(
                period=7.0,
                actions=[
                    ExecuteProcess(
                        cmd=["python3", bird_manager_py],
                        output="screen",
                        additional_env={"PYTHONUNBUFFERED": "1"},
                    )
                ],
                condition=IfCondition(LaunchConfiguration("enable_ugv_bird_manager")),
            ),
            Node(
                package="waver_patrol",
                executable="gazebo_bird_pose_bridge_node",
                name="gazebo_bird_pose_bridge_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_gazebo_bird_bridge")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "pose_array_topic": "/waver/lidar_objects",
                        "pose_array_frame_id": "map",
                        "publish_pose_array": True,
                        "publish_object_point": False,
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="fake_camera_classification_test_publisher_node",
                name="fake_camera_classification_test_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_fake_camera_classification")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "publish_after_sec": 8.0,
                        "target_class": "bird",
                        "confidence": 0.92,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_launch),
                condition=IfCondition(LaunchConfiguration("enable_mission_stack")),
                launch_arguments={
                    "use_nav2": LaunchConfiguration("use_nav2"),
                    "require_scan": LaunchConfiguration("require_scan"),
                    "enable_test_publishers": "false",
                    "enable_deep_learning_stub": LaunchConfiguration("enable_fake_camera_classification"),
                    "enable_sound_stub": LaunchConfiguration("enable_fake_sound"),
                    "start_serial_bridge": "false",
                    "enable_experiment_logger": "false",
                    "waypoint_file": LaunchConfiguration("waypoint_file"),
                    "default_mode": default_mode,
                    "target_classification_timeout_sec": "8.0",
                    "mission_sound_task_timeout_sec": "4.0",
                    "sound_task_duration_sec": "1.0",
                    "sound_alert_cooldown_sec": "0.5",
                    "sound_done_latch_sec": "3.0",
                    "post_target_resume_cooldown_sec": LaunchConfiguration("post_target_resume_cooldown_sec"),
                    "use_sim_time": "true",
                    "enable_sim_nav_goal_arrival": "true",
                    "require_robot_pose_for_goal": "false",
                    "enable_moving_object_map_transform": "true",
                    "ignore_scan_when_require_scan_false": "true",
                    "safety_max_linear_speed": LaunchConfiguration("gazebo_sim_max_linear_speed"),
                    "safety_max_angular_speed": "1.0",
                    "moving_object_input_topic": "/waver/lidar_objects",
                    "moving_object_input_type": "pose_array",
                    "moving_object_output_topic": "/waver/lidar_objects_map",
                    "moving_object_marker_topic": "/waver/moving_objects_map_marker",
                }.items(),
            ),
            Node(
                package="waver_patrol",
                executable="gazebo_map_path_visualizer_node",
                name="gazebo_map_path_visualizer_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_gazebo_map_path_visualizer")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "map_yaml": LaunchConfiguration("map_yaml"),
                        "map_topic": "/map",
                        "global_path_topic": "/plan",
                        "local_path_topic": "/local_plan",
                        "odom_topic": "/odom",
                        "active_goal_topic": "/waver/active_nav_goal",
                        "current_waypoint_topic": "/waver/current_waypoint",
                        "dynamic_obstacle_topic": LaunchConfiguration("dynamic_obstacle_topic"),
                        "enable_dynamic_obstacle_detour": ParameterValue(
                            LaunchConfiguration("enable_dynamic_obstacle_detour"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="simple_nav2_cmd_sim_node",
                name="simple_nav2_cmd_sim_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_simple_nav2_cmd_sim")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "max_linear_speed": ParameterValue(LaunchConfiguration("gazebo_sim_max_linear_speed"), value_type=float),
                        "max_angular_speed": 1.0,
                        "goal_tolerance_m": ParameterValue(
                            LaunchConfiguration("gazebo_goal_tolerance_m"),
                            value_type=float,
                        ),
                        "yaw_tolerance_rad": 0.75,
                        "goal_timeout_sec": 240.0,
                        "dynamic_obstacle_topic": LaunchConfiguration("dynamic_obstacle_topic"),
                        "enable_dynamic_obstacle_avoidance": ParameterValue(
                            LaunchConfiguration("enable_simple_nav2_avoidance"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="moving_object_motion_filter_node",
                name="moving_object_motion_filter_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_moving_object_motion_filter")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "input_topic": "/waver/lidar_objects_map",
                        "raw_input_topic": "/waver/lidar_objects",
                        "target_min_height_m": ParameterValue(target_min_height, value_type=float),
                        "target_max_height_m": 30.0,
                        "height_reference_frame": "map",
                        "fallback_height_reference_frame": "odom",
                        "ground_z_offset_m": 0.0,
                        "require_z_valid": True,
                        "require_height_filter": True,
                        "require_dynamic_filter": True,
                        "min_dynamic_motion_m": ParameterValue(min_dynamic_motion, value_type=float),
                        "min_dynamic_velocity_mps": ParameterValue(min_dynamic_velocity, value_type=float),
                        "min_sample_motion_epsilon_m": 0.005,
                        "min_tracking_duration_sec": 1.0,
                        "max_tracking_duration_sec": 20.0,
                        "require_consecutive_dynamic_frames": 5,
                        "static_motion_tolerance_m": 0.25,
                        "high_yaw_rate_mode": "strict_validation",
                        "track_match_gate_m": 0.9,
                        "max_sample_step_m": 1.25,
                        "enable_range_filter": False,
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="gazebo_moving_object_trial_publisher_node",
                name="gazebo_moving_object_trial_publisher_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_cluster_node")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "trial_id": ParameterValue(trial_id, value_type=int),
                        "scenario_id": ParameterValue(
                            LaunchConfiguration("target_scenario_id"),
                            value_type=int,
                        ),
                        "duration_sec": ParameterValue(LaunchConfiguration("target_motion_duration_sec"), value_type=float),
                        "target_z": ParameterValue(LaunchConfiguration("target_z"), value_type=float),
                        "gazebo_entity_move_delay_sec": ParameterValue(
                            LaunchConfiguration("target_gazebo_move_delay_sec"),
                            value_type=float,
                        ),
                        "start_on_mission_command": ParameterValue(
                            LaunchConfiguration("target_start_on_mission_command"),
                            value_type=bool,
                        ),
                        "frame_id": "map",
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="gazebo_trial_logger_node",
                name="gazebo_trial_logger_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_trial_logger")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "trial_id": ParameterValue(trial_id, value_type=int),
                        "target_min_height_m": ParameterValue(target_min_height, value_type=float),
                        "min_dynamic_motion_m": ParameterValue(min_dynamic_motion, value_type=float),
                        "output_root": output_root,
                    }
                ],
            ),
            TimerAction(
                period=6.0,
                actions=[
                    ExecuteProcess(
                        cmd=["bash", "-lc", rosbag_command_text],
                        output="screen",
                        condition=IfCondition(record_bag),
                    )
                ],
            ),
        ]
    )
