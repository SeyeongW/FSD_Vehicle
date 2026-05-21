from __future__ import annotations

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _find_ugv_script(filename: str, ugv_share: str) -> str:
    installed = os.path.normpath(os.path.join(ugv_share, "..", "..", "lib", "ugv_gazebo", filename))
    if os.path.isfile(installed):
        return installed
    source_matches = glob.glob(os.path.expanduser(f"~/ros2_ws/src/**/{filename}"), recursive=True)
    return source_matches[0] if source_matches else installed


def launch_setup(context, *args, **kwargs):
    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    ugv_description_share = get_package_share_directory("ugv_description")

    model = LaunchConfiguration("ugv_model").perform(context)
    world_arg = LaunchConfiguration("world").perform(context)
    world = world_arg if os.path.isabs(world_arg) else os.path.join(ugv_share, "worlds", world_arg)
    model_sdf = os.path.join(ugv_share, "models", model, "model.sdf")
    urdf = os.path.join(ugv_share, "urdf", f"{model}.urdf")
    bird_model = os.path.join(ugv_share, "models", "bird", "model.sdf")
    bird_manager = _find_ugv_script("bird_manager.py", ugv_share)
    waver_launch = os.path.join(waver_share, "launch", "waver_bird_autonomy.launch.py")

    return [
        SetEnvironmentVariable(name="GAZEBO_MODEL_DATABASE_URI", value=""),
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=":".join(
                path
                for path in [
                    os.path.join(ugv_share, "models"),
                    os.path.dirname(ugv_description_share),
                    "/usr/share/gazebo-11/models",
                    os.environ.get("GAZEBO_MODEL_PATH", ""),
                ]
                if path
            ),
        ),
        SetEnvironmentVariable(
            name="GAZEBO_RESOURCE_PATH",
            value=":".join(
                path
                for path in [
                    "/usr/share/gazebo-11",
                    "/usr/share/gazebo-11/media",
                    os.environ.get("GAZEBO_RESOURCE_PATH", ""),
                ]
                if path
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
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            arguments=[urdf],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity",
                model,
                "-file",
                model_sdf,
                "-x",
                LaunchConfiguration("x_pose"),
                "-y",
                LaunchConfiguration("y_pose"),
                "-z",
                "0.05",
            ],
            output="screen",
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=["-entity", "bird_single", "-file", bird_model, "-x", "3.0", "-y", "2.0", "-z", "6.0"],
                    output="screen",
                )
            ],
            condition=IfCondition(LaunchConfiguration("spawn_birds")),
        ),
        TimerAction(
            period=4.8,
            actions=[
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=["-entity", "bird_swarm_1", "-file", bird_model, "-x", "-6.0", "-y", "0.0", "-z", "6.5"],
                    output="screen",
                ),
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=["-entity", "bird_swarm_2", "-file", bird_model, "-x", "6.0", "-y", "0.0", "-z", "6.5"],
                    output="screen",
                ),
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=["-entity", "bird_swarm_3", "-file", bird_model, "-x", "-3.0", "-y", "-5.5", "-z", "7.0"],
                    output="screen",
                ),
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=["-entity", "bird_swarm_4", "-file", bird_model, "-x", "3.0", "-y", "-5.5", "-z", "5.8"],
                    output="screen",
                ),
                Node(
                    package="gazebo_ros",
                    executable="spawn_entity.py",
                    arguments=["-entity", "bird_swarm_5", "-file", bird_model, "-x", "0.0", "-y", "6.0", "-z", "6.8"],
                    output="screen",
                ),
            ],
            condition=IfCondition(LaunchConfiguration("spawn_birds")),
        ),
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=["python3", bird_manager],
                    output="screen",
                    additional_env={"PYTHONUNBUFFERED": "1"},
                )
            ],
            condition=IfCondition(LaunchConfiguration("spawn_birds")),
        ),
        Node(
            package="waver_patrol",
            executable="livox_pointcloud_to_scan_node",
            name="livox_pointcloud_to_scan_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_livox_scan_adapter")),
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "use_sim_time": True,
                    "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                    "scan_topic": "/scan",
                    "forward_axis": LaunchConfiguration("scan_forward_axis"),
                    "lateral_axis": LaunchConfiguration("scan_lateral_axis"),
                    "height_axis": LaunchConfiguration("scan_height_axis"),
                    "positive_lateral_is_left": ParameterValue(LaunchConfiguration("scan_positive_lateral_is_left"), value_type=bool),
                },
            ],
        ),
        Node(
            package="waver_patrol",
            executable="gazebo_bird_pose_bridge_node",
            name="gazebo_bird_pose_bridge_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_gazebo_bird_bridge")),
            parameters=[LaunchConfiguration("config_file"), {"use_sim_time": True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(waver_launch),
            launch_arguments={
                "config_file": LaunchConfiguration("config_file"),
                "waypoint_file": LaunchConfiguration("waypoint_file"),
                "use_sim_time": "true",
                "use_sim_odom": "false",
                "start_serial_bridge": "false",
                "enable_keyboard_teleop": "false",
                "enable_test_publishers": "false",
                "require_scan": LaunchConfiguration("require_scan"),
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
        ),
        Node(
            package="ugv_tools",
            executable="keyboard_ctrl",
            name="waver_scripted_keyboard_test",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_scripted_keyboard_test")),
            arguments=[
                "--scripted-keys",
                LaunchConfiguration("scripted_keys"),
                "--scripted-interval",
                LaunchConfiguration("scripted_interval"),
            ],
            parameters=[
                {
                    "use_sim_time": True,
                    "cmd_vel_topic": "/waver/manual_cmd_vel",
                    "output_mode": "twist",
                    "enable_scan_assist": True,
                    "lidar_required": False,
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    waver_share = get_package_share_directory("waver_patrol")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=os.path.join(waver_share, "config", "waver_bird_autonomy.yaml")),
            DeclareLaunchArgument("waypoint_file", default_value=os.path.join(waver_share, "waypoints", "waver_bird_patrol_demo.yaml")),
            DeclareLaunchArgument("ugv_model", default_value="ugv_rover"),
            DeclareLaunchArgument("world", default_value="waver_empty.world"),
            DeclareLaunchArgument("use_gui", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("x_pose", default_value="0.0"),
            DeclareLaunchArgument("y_pose", default_value="0.0"),
            DeclareLaunchArgument("spawn_birds", default_value="true"),
            DeclareLaunchArgument("enable_livox_scan_adapter", default_value="true"),
            DeclareLaunchArgument("enable_gazebo_bird_bridge", default_value="true"),
            DeclareLaunchArgument("pointcloud_topic", default_value="/camera/points"),
            DeclareLaunchArgument("scan_forward_axis", default_value="z"),
            DeclareLaunchArgument("scan_lateral_axis", default_value="x"),
            DeclareLaunchArgument("scan_height_axis", default_value="-y"),
            DeclareLaunchArgument("scan_positive_lateral_is_left", default_value="false"),
            DeclareLaunchArgument("require_scan", default_value="true"),
            DeclareLaunchArgument("enable_scripted_keyboard_test", default_value="false"),
            DeclareLaunchArgument("scripted_keys", default_value="wwk"),
            DeclareLaunchArgument("scripted_interval", default_value="0.18"),
            OpaqueFunction(function=launch_setup),
        ]
    )
