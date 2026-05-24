from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Start only Gazebo + ugv_rover + passive map/path visualization.

    역할:
      - 사용자가 Gazebo를 켰을 때 로버가 자동으로 움직이지 않게 하는 순수 시뮬레이터 launch다.
      - fake target, moving-object publisher, trial logger를 모두 끈다.
      - safety/mission backend는 STANDBY로만 켜서 `/cmd_vel`은 0으로 유지한다.
      - 실제 이동은 별도 터미널의 `ugv_tools waver_operator_panel.launch.py`에서
        AUTO/키보드/STOP을 눌렀을 때만 시작한다.
    """

    waver_share = get_package_share_directory("waver_patrol")
    ugv_share = get_package_share_directory("ugv_gazebo")
    trial_launch = os.path.join(waver_share, "launch", "gazebo_moving_object_trial.launch.py")
    default_world = os.path.join(ugv_share, "worlds", "ugv_world.world")
    default_robot = os.path.join(ugv_share, "models", "ugv_rover", "model.sdf")
    default_map = os.path.join(ugv_share, "maps", "map.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("world_file", default_value=default_world),
            DeclareLaunchArgument("robot_sdf_file", default_value=default_robot),
            DeclareLaunchArgument("map_yaml", default_value=default_map),
            DeclareLaunchArgument("robot_spawn_x", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_spawn_z", default_value="0.15"),
            DeclareLaunchArgument("robot_spawn_yaw", default_value="0.0"),
            LogInfo(
                msg=(
                    "Waver Gazebo rover-only mode: spawns ugv_rover and publishes "
                    "passive /map,/plan,/local_plan preview only. No autonomy or fake target; "
                    "safety_cmd_mux_node publishes final /cmd_vel=0 until the remote panel sends manual/AUTO commands."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(trial_launch),
                launch_arguments={
                    "use_gui": LaunchConfiguration("use_gui"),
                    "world_file": LaunchConfiguration("world_file"),
                    "robot_entity": "ugv_rover",
                    "robot_sdf_file": LaunchConfiguration("robot_sdf_file"),
                    "robot_spawn_x": LaunchConfiguration("robot_spawn_x"),
                    "robot_spawn_y": LaunchConfiguration("robot_spawn_y"),
                    "robot_spawn_z": LaunchConfiguration("robot_spawn_z"),
                    "robot_spawn_yaw": LaunchConfiguration("robot_spawn_yaw"),
                    "map_yaml": LaunchConfiguration("map_yaml"),
                    "spawn_robot": "true",
                    "spawn_target": "false",
                    "spawn_ugv_bird_single": "false",
                    "spawn_ugv_bird_swarm": "false",
                    "enable_ugv_bird_manager": "false",
                    "enable_gazebo_bird_bridge": "false",
                    "enable_fake_camera_classification": "false",
                    "enable_mission_stack": "true",
                    "enable_simple_nav2_cmd_sim": "true",
                    "enable_moving_object_motion_filter": "false",
                    "enable_cluster_node": "false",
                    "enable_trial_logger": "false",
                    "record_bag": "false",
                    "require_scan": "false",
                    "enable_gazebo_map_path_visualizer": "true",
                    "default_mode": "STANDBY",
                }.items(),
            ),
        ]
    )
