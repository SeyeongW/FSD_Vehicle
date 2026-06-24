from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def _package_prefix(package_name: str) -> str:
    try:
        return get_package_prefix(package_name)
    except Exception:
        return ""


def _resolve_world(world_arg: str, ugv_share: str) -> str:
    expanded = os.path.expanduser(os.path.expandvars(world_arg))
    if os.path.isabs(expanded):
        return expanded
    candidate = os.path.join(ugv_share, "worlds", expanded)
    if os.path.exists(candidate):
        return candidate
    return os.path.join(ugv_share, "worlds", "ugv_world.world")


def _launch_setup(context, *args, **kwargs):
    ugv_share = get_package_share_directory("ugv_gazebo")
    waver_share = get_package_share_directory("waver_patrol")
    ugv_description_parent = os.path.dirname(get_package_share_directory("ugv_description"))
    gazebo_ros_prefix = _package_prefix("gazebo_ros")
    gazebo_ros_lib = os.path.join(gazebo_ros_prefix, "lib") if gazebo_ros_prefix else ""
    ws_root = os.path.expanduser("~/ros2_ws5/FSD_Vehicle")

    gazebo_model_path = ":".join(
        p
        for p in [
            os.path.join(ugv_share, "models"),
            ugv_description_parent,
            os.environ.get("GAZEBO_MODEL_PATH", ""),
        ]
        if p
    )
    gazebo_resource_path = ":".join(
        p
        for p in [
            os.path.join(ugv_share, "worlds"),
            os.path.join(ugv_share, "models"),
            "/usr/share/gazebo-11",
            "/usr/share/gazebo",
            os.environ.get("GAZEBO_RESOURCE_PATH", ""),
        ]
        if p
    )
    gazebo_plugin_path = ":".join(
        p for p in [gazebo_ros_lib, os.environ.get("GAZEBO_PLUGIN_PATH", "")] if p
    )

    world = _resolve_world(LaunchConfiguration("world").perform(context), ugv_share)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_sdf = os.path.join(ugv_share, "models", robot_model, "model.sdf")
    robot_urdf = os.path.join(ugv_share, "urdf", f"{robot_model}.urdf")
    ui_slam_params = os.path.join(ugv_share, "param", "ui_slam", "gazebo.yaml")
    nav2_params = LaunchConfiguration("nav2_params_file")
    waypoint_file = os.path.join(waver_share, "waypoints", "waver_nav2_patrol_mission.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("use_gui")
    map_file = LaunchConfiguration("map")
    start_remote_panel = LaunchConfiguration("start_remote_panel")
    common_params = [{"use_sim_time": use_sim_time}]
    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            param_rewrites={"use_sim_time": use_sim_time, "yaml_filename": map_file},
            convert_types=True,
        ),
        allow_substs=True,
    )
    lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    return [
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
        SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", gazebo_resource_path),
        SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", gazebo_plugin_path),
        ExecuteProcess(
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
        ),
        ExecuteProcess(cmd=["gzclient"], output="screen", condition=IfCondition(use_gui)),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            arguments=[robot_urdf],
            parameters=common_params,
            output="screen",
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name="spawn_ugv_rover",
                    arguments=[
                        "-entity",
                        robot_model,
                        "-file",
                        robot_sdf,
                        "-x",
                        LaunchConfiguration("robot_spawn_x"),
                        "-y",
                        LaunchConfiguration("robot_spawn_y"),
                        "-z",
                        LaunchConfiguration("robot_spawn_z"),
                    ],
                    output="screen",
                )
            ],
        ),
        TimerAction(
            period=3.5,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name="spawn_static_nav_obstacle",
                    arguments=[
                        "-entity",
                        LaunchConfiguration("static_obstacle_entity"),
                        "-file",
                        LaunchConfiguration("static_obstacle_sdf"),
                        "-x",
                        LaunchConfiguration("static_obstacle_x"),
                        "-y",
                        LaunchConfiguration("static_obstacle_y"),
                        "-z",
                        LaunchConfiguration("static_obstacle_z"),
                    ],
                    output="screen",
                )
            ],
            condition=IfCondition(LaunchConfiguration("spawn_static_obstacle")),
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="waver_patrol",
                    executable="scan_republisher_node",
                    name="scan_to_safety_alias",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "input_topic": "/scan",
                            "output_topic": "/scan_safety",
                        }
                    ],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="waver_patrol",
                    executable="mission_patrol_manager_node",
                    name="mission_patrol_manager_node",
                    parameters=[
                        ui_slam_params,
                        {
                            "use_nav2": ParameterValue(
                                LaunchConfiguration("mission_manager_use_nav2"),
                                value_type=bool,
                            ),
                            "waypoint_file": waypoint_file,
                            "default_mode": "STANDBY",
                            "enable_sim_nav_goal_arrival": False,
                        },
                        *common_params,
                    ],
                    output="screen",
                ),
                Node(
                    package="waver_patrol",
                    executable="safety_cmd_mux_node",
                    name="safety_cmd_mux_node",
                    parameters=[ui_slam_params, *common_params],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="map_server",
                    output="screen",
                    parameters=[configured_nav2_params],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("nav2_log_level")],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
                Node(
                    package="waver_patrol",
                    executable="static_map_state_publisher_node",
                    name="static_map_state_publisher_node",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "map_topic": "/map",
                            "map_path": map_file,
                            "publish_period_sec": 0.5,
                        }
                    ],
                ),
                Node(
                    package="nav2_amcl",
                    executable="amcl",
                    name="amcl",
                    output="screen",
                    parameters=[configured_nav2_params],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("nav2_log_level")],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
                Node(
                    package="nav2_controller",
                    executable="controller_server",
                    name="controller_server",
                    output="screen",
                    parameters=[configured_nav2_params],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("nav2_log_level")],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "/waver/cmd_vel_nav2")],
                ),
                Node(
                    package="nav2_planner",
                    executable="planner_server",
                    name="planner_server",
                    output="screen",
                    parameters=[configured_nav2_params],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("nav2_log_level")],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
                Node(
                    package="nav2_behaviors",
                    executable="behavior_server",
                    name="behavior_server",
                    output="screen",
                    parameters=[configured_nav2_params],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("nav2_log_level")],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static"), ("cmd_vel", "/waver/cmd_vel_nav2")],
                ),
                Node(
                    package="nav2_bt_navigator",
                    executable="bt_navigator",
                    name="bt_navigator",
                    output="screen",
                    parameters=[configured_nav2_params],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("nav2_log_level")],
                    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_nav2_saved_map",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"autostart": True},
                        {"node_names": lifecycle_nodes},
                    ],
                    arguments=["--ros-args", "--log-level", LaunchConfiguration("nav2_log_level")],
                )
            ],
        ),
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package="ugv_tools",
                    executable="waver_remote_panel",
                    name="waver_remote_panel",
                    output="screen",
                    parameters=[
                        ui_slam_params,
                        {
                            "demo_script": LaunchConfiguration("remote_panel_demo_script"),
                            "demo_close_on_finish": LaunchConfiguration("demo_close_on_finish"),
                            "publish_direct_cmd_vel": False,
                            "allow_subprocess_launches": False,
                            "allow_mapping_launches": False,
                            "allow_map_save_commands": False,
                            "allow_localization_launches": False,
                        },
                        *common_params,
                    ],
                    condition=IfCondition(start_remote_panel),
                )
            ],
        ),
    ]


def generate_launch_description():
    waver_share = get_package_share_directory("waver_patrol")
    default_static_obstacle_sdf = os.path.join(waver_share, "models", "static_obstacle_box", "model.sdf")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("world", default_value="ugv_world.world"),
            DeclareLaunchArgument("robot_model", default_value="ugv_rover"),
            DeclareLaunchArgument("robot_spawn_x", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_z", default_value="0.15"),
            DeclareLaunchArgument("spawn_static_obstacle", default_value="false"),
            DeclareLaunchArgument("static_obstacle_entity", default_value="nav_static_test_box"),
            DeclareLaunchArgument("static_obstacle_sdf", default_value=default_static_obstacle_sdf),
            DeclareLaunchArgument("static_obstacle_x", default_value="1.00"),
            DeclareLaunchArgument("static_obstacle_y", default_value="0.0"),
            DeclareLaunchArgument("static_obstacle_z", default_value="0.0"),
            DeclareLaunchArgument("start_remote_panel", default_value="false"),
            DeclareLaunchArgument(
                "mission_manager_use_nav2",
                default_value="false",
                description=(
                    "Keep false for direct Nav2/localization validation so the mission manager "
                    "does not inject patrol waypoint goals. Set true only when testing START_PATROL."
                ),
            ),
            DeclareLaunchArgument("remote_panel_demo_script", default_value=""),
            DeclareLaunchArgument("demo_close_on_finish", default_value="false"),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.expanduser("~/ros2_ws5/FSD_Vehicle/maps/waver_latest_map.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.expanduser(
                    "~/ros2_ws5/FSD_Vehicle/src/ugv_main/ugv_gazebo/param/amcl_dwa.yaml"
                ),
            ),
            DeclareLaunchArgument("nav2_log_level", default_value="warn"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
