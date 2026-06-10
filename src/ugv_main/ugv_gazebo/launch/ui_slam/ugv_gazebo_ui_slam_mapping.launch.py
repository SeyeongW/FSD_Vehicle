from __future__ import annotations

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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
    ws_root = os.path.expanduser("~/ros2_ws3/FSD_Vehicle")
    livox_prefix = _package_prefix("ros2_livox_simulation")
    livox_plugin_candidates = [
        os.path.join(livox_prefix, "lib") if livox_prefix else "",
        os.path.join(ws_root, "install", "ros2_livox_simulation", "lib"),
        os.path.join(ws_root, "build", "ros2_livox_simulation"),
    ]
    gazebo_plugin_path = ":".join(
        p for p in [gazebo_ros_lib, *livox_plugin_candidates, os.environ.get("GAZEBO_PLUGIN_PATH", "")] if p
    )
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

    world = _resolve_world(LaunchConfiguration("world").perform(context), ugv_share)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_sdf = os.path.join(ugv_share, "models", robot_model, "model.sdf")
    robot_urdf = os.path.join(ugv_share, "urdf", f"{robot_model}.urdf")
    ui_slam_params = os.path.join(ugv_share, "param", "ui_slam", "gazebo.yaml")
    rviz_config = os.path.join(ugv_share, "rviz", "view_nav_2d.rviz")
    waypoint_file = os.path.join(waver_share, "waypoints", "waver_4m_then_7m_square_patrol.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_gui = LaunchConfiguration("use_gui")
    start_remote_panel = LaunchConfiguration("start_remote_panel")
    start_patrol_node = LaunchConfiguration("start_patrol_node")
    start_slam = LaunchConfiguration("start_slam")
    backend = LaunchConfiguration("mapping_backend")
    common_params = [{"use_sim_time": use_sim_time}]

    slam_toolbox_condition = IfCondition(PythonExpression(["'", backend, "' == 'slam_toolbox' and '", start_slam, "' == 'true'"]))
    scan_mapper_condition = IfCondition(PythonExpression(["'", backend, "' == 'scan_mapper' and '", start_slam, "' == 'true'"]))
    fake_map_condition = IfCondition(PythonExpression(["'", backend, "' == 'gazebo_live' and '", start_slam, "' == 'true'"]))

    static_obstacle_field = [
        ("mapping_static_obs_01", "1.0", "1.3"),
        ("mapping_static_obs_02", "2.3", "-1.3"),
        ("mapping_static_obs_03", "-1.1", "1.1"),
        ("mapping_static_obs_04", "-1.4", "-0.9"),
        ("mapping_static_obs_05", "3.0", "1.6"),
        ("mapping_static_obs_06", "-2.0", "0.4"),
    ]
    static_obstacle_field_actions = [
        TimerAction(
            period=3.8 + 0.25 * index,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    name=f"spawn_{entity}",
                    arguments=[
                        "-entity",
                        entity,
                        "-file",
                        LaunchConfiguration("static_obstacle_sdf"),
                        "-x",
                        x,
                        "-y",
                        y,
                        "-z",
                        LaunchConfiguration("static_obstacle_z"),
                    ],
                    output="screen",
                )
            ],
            condition=IfCondition(LaunchConfiguration("spawn_static_obstacle_field")),
        )
        for index, (entity, x, y) in enumerate(static_obstacle_field)
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
                    name="spawn_mapping_static_obstacle",
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
        *static_obstacle_field_actions,
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="waver_patrol",
                    executable="scan_republisher_node",
                    name="scan_to_slam_alias",
                    parameters=[{"use_sim_time": use_sim_time, "input_topic": "/scan", "output_topic": "/scan_slam"}],
                    output="screen",
                ),
                Node(
                    package="waver_patrol",
                    executable="scan_republisher_node",
                    name="scan_to_safety_alias",
                    parameters=[{"use_sim_time": use_sim_time, "input_topic": "/scan", "output_topic": "/scan_safety"}],
                    output="screen",
                ),
            ],
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="slam_toolbox",
                    parameters=[ui_slam_params, *common_params],
                    output="screen",
                    condition=slam_toolbox_condition,
                ),
                Node(
                    package="waver_patrol",
                    executable="laser_scan_occupancy_mapper_node",
                    name="laser_scan_occupancy_mapper_node",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "scan_topic": "/scan_slam",
                            "odom_topic": "/odom",
                            "map_topic": "/map",
                            "map_frame": "map",
                            "odom_frame": "odom",
                            "base_frame": "base_footprint",
                            "resolution": 0.05,
                            "extent_m": 30.0,
                            "center_x": 0.0,
                            "center_y": 0.0,
                            "max_range_m": 12.0,
                            "ray_step": 2,
                            "occupied_inflate_cells": 1,
                            "publish_rate_hz": 2.0,
                            "publish_map_to_odom_tf": True,
                        },
                        *common_params,
                    ],
                    output="screen",
                    condition=scan_mapper_condition,
                ),
                Node(
                    package="waver_patrol",
                    executable="gazebo_live_mapping_node",
                    name="gazebo_live_mapping_node",
                    parameters=[
                        {
                            "use_sim_time": use_sim_time,
                            "map_topic": "/map",
                            "map_fixed_topic": "/map_fixed",
                            "map_path": os.path.join(ws_root, "maps", "waver_latest_map.yaml"),
                            "publish_period_sec": 1.0,
                        }
                    ],
                    output="screen",
                    condition=fake_map_condition,
                ),
            ],
        ),
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="waver_patrol",
                    executable="mapping_workflow_manager_node",
                    name="mapping_workflow_manager_node",
                    parameters=[ui_slam_params, {"save_dir": LaunchConfiguration("save_dir")}, *common_params],
                    output="screen",
                ),
                Node(
                    package="waver_patrol",
                    executable="mapping_backend_manager_node",
                    name="mapping_backend_manager_node",
                    parameters=[ui_slam_params, *common_params],
                    output="screen",
                ),
                Node(
                    package="waver_patrol",
                    executable="mission_patrol_manager_node",
                    name="mission_patrol_manager_node",
                    parameters=[ui_slam_params, {"waypoint_file": waypoint_file, "use_nav2": False}, *common_params],
                    output="screen",
                ),
                Node(
                    package="waver_patrol",
                    executable="safety_cmd_mux_node",
                    name="safety_cmd_mux_node",
                    parameters=[ui_slam_params, *common_params],
                    output="screen",
                ),
                Node(
                    package="ugv_tools",
                    executable="waver_gazebo_patrol",
                    name="waver_gazebo_patrol",
                    parameters=[
                        ui_slam_params,
                        {
                            "waypoint_file": waypoint_file,
                            "cmd_vel_topic": "/waver/cmd_vel_nav2",
                            "odom_topic": "/odom",
                            "scan_topic": "/scan_safety",
                        },
                        *common_params,
                    ],
                    output="screen",
                    condition=IfCondition(start_patrol_node),
                ),
            ],
        ),
        TimerAction(
            # Gazebo Classic can take 15-20s to spawn the rover and load the
            # diff-drive plugin on slower machines.  Starting the operator UI
            # after that avoids early keyboard/demo commands being published
            # before /cmd_vel has a real Gazebo subscriber.
            period=22.0,
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
        TimerAction(
            period=24.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2_ui_slam",
                    arguments=["-d", rviz_config],
                    parameters=common_params,
                    output="screen",
                    condition=IfCondition(LaunchConfiguration("start_rviz")),
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
            DeclareLaunchArgument("mapping_backend", default_value="scan_mapper"),
            DeclareLaunchArgument("start_slam", default_value="true"),
            DeclareLaunchArgument("start_remote_panel", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="false"),
            DeclareLaunchArgument("remote_panel_demo_script", default_value=""),
            DeclareLaunchArgument("demo_close_on_finish", default_value="false"),
            DeclareLaunchArgument("start_patrol_node", default_value="true"),
            DeclareLaunchArgument("save_dir", default_value=os.path.expanduser("~/ros2_ws3/FSD_Vehicle/maps")),
            DeclareLaunchArgument("spawn_static_obstacle", default_value="false"),
            DeclareLaunchArgument("spawn_static_obstacle_field", default_value="false"),
            DeclareLaunchArgument("static_obstacle_entity", default_value="mapping_static_test_box"),
            DeclareLaunchArgument("static_obstacle_sdf", default_value=default_static_obstacle_sdf),
            DeclareLaunchArgument("static_obstacle_x", default_value="1.0"),
            DeclareLaunchArgument("static_obstacle_y", default_value="0.0"),
            DeclareLaunchArgument("static_obstacle_z", default_value="0.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
