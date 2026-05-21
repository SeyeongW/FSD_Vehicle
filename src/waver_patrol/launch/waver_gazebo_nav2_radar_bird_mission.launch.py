from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    ugv_tools_share = get_package_share_directory("ugv_tools")
    mission_launch = os.path.join(waver_share, "launch", "waver_nav2_radar_bird_mission.launch.py")

    world = LaunchConfiguration("world")
    model = LaunchConfiguration("model")
    model_sdf = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")
    minimal_model_sdf = os.path.join(ugv_tools_share, "models", "waver_safety_sim", "model.sdf")
    bird_sdf = os.path.join(ugv_share, "models", "bird", "model.sdf")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=os.path.join(ugv_tools_share, "worlds", "waver_flat.world")),
            DeclareLaunchArgument("model", default_value="ugv_rover"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            DeclareLaunchArgument("use_minimal_model", default_value="true"),
            DeclareLaunchArgument("spawn_birds", default_value="true"),
            DeclareLaunchArgument("require_scan", default_value="false"),
            DeclareLaunchArgument("enable_test_publishers", default_value="true"),
            DeclareLaunchArgument("enable_simple_nav2_cmd_sim", default_value="true"),
            DeclareLaunchArgument("enable_dynamic_obstacle", default_value="true"),
            DeclareLaunchArgument("enable_remote_panel", default_value="false"),
            DeclareLaunchArgument("remote_demo_script", default_value=""),
            DeclareLaunchArgument("remote_demo_close_on_finish", default_value="false"),
            DeclareLaunchArgument("enable_experiment_logger", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="false"),
            SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI", value=""),
            SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
            SetEnvironmentVariable(name="GAZEBO_RESOURCE_PATH", value="/usr/share/gazebo-11"),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH",
                value=(
                    f"{os.path.join(ugv_share, 'models')}:"
                    f"{os.path.join(ugv_tools_share, 'models')}:"
                    f"/usr/share/gazebo-11/models:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
                ),
            ),
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
            ExecuteProcess(cmd=["gzclient"], output="screen", condition=IfCondition(LaunchConfiguration("use_gui"))),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=["-entity", model, "-file", model_sdf, "-x", "0.0", "-y", "0.0", "-z", "0.05"],
                        output="screen",
                    )
                ],
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration("spawn_robot"), "' == 'true' and '", LaunchConfiguration("use_minimal_model"), "' == 'false'"])),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=["-entity", "waver_safety_sim", "-file", minimal_model_sdf, "-x", "0.0", "-y", "0.0", "-z", "0.05"],
                        output="screen",
                    )
                ],
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration("spawn_robot"), "' == 'true' and '", LaunchConfiguration("use_minimal_model"), "' == 'true'"])),
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="gazebo_ros",
                        executable="spawn_entity.py",
                        arguments=["-entity", "bird_test_target", "-file", bird_sdf, "-x", "2.0", "-y", "1.0", "-z", "4.0"],
                        output="screen",
                    )
                ],
                condition=IfCondition(LaunchConfiguration("spawn_birds")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mission_launch),
                launch_arguments={
                    "use_nav2": LaunchConfiguration("use_nav2"),
                    "require_scan": LaunchConfiguration("require_scan"),
                    "enable_test_publishers": LaunchConfiguration("enable_test_publishers"),
                    "start_serial_bridge": "false",
                    "enable_experiment_logger": LaunchConfiguration("enable_experiment_logger"),
                    "default_mode": "AUTO",
                    "use_sim_time": "true",
                    "enable_sim_nav_goal_arrival": "true",
                }.items(),
            ),
            Node(
                package="waver_patrol",
                executable="simple_nav2_cmd_sim_node",
                name="simple_nav2_cmd_sim_node",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            LaunchConfiguration("enable_simple_nav2_cmd_sim"),
                            "' == 'true' and '",
                            LaunchConfiguration("use_nav2"),
                            "' == 'false'",
                        ]
                    )
                ),
                parameters=[
                    {
                        "use_sim_time": True,
                        "max_linear_speed": 0.16,
                        "max_angular_speed": 0.45,
                        # Gazebo-only Nav2 stand-in: match the loose outdoor/Nav2 goal tolerance.
                        "goal_tolerance_m": 0.45,
                    }
                ],
            ),
            Node(
                package="waver_patrol",
                executable="gazebo_dynamic_obstacle_node",
                name="gazebo_dynamic_obstacle_node",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_dynamic_obstacle")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "entity_name": "dynamic_test_box",
                        "x_m": 0.58,
                        "y_amplitude_m": 1.15,
                        "period_sec": 5.0,
                    }
                ],
            ),
            Node(
                package="ugv_tools",
                executable="waver_remote_panel",
                name="waver_remote_panel",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_remote_panel")),
                parameters=[
                    {
                        "use_sim_time": True,
                        "cmd_vel_topic": "/cmd_vel",
                        "manual_cmd_vel_topic": "/waver/manual_cmd_vel",
                        "auto_cmd_vel_topic": "/waver/cmd_vel_nav2",
                        "publish_direct_cmd_vel": False,
                        "manual_override_returns_to_auto": True,
                        "auto_mode_strategy": "mission_nav2",
                        "lidar_required": False,
                        "demo_script": LaunchConfiguration("remote_demo_script"),
                        "demo_close_on_finish": LaunchConfiguration("remote_demo_close_on_finish"),
                    }
                ],
            ),
        ]
    )
