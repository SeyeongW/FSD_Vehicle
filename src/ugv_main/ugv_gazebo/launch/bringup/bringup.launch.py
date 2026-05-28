import os
import glob as _glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_script(filename: str, pkg_share: str) -> str:
    candidate = os.path.normpath(
        os.path.join(pkg_share, '..', '..', 'lib', 'ugv_gazebo', filename)
    )
    if os.path.isfile(candidate):
        return candidate

    ws_src = os.path.expanduser('~/ugv_ws/src')
    matches = _glob.glob(os.path.join(ws_src, '**', filename), recursive=True)
    if matches:
        return matches[0]

    cwd_matches = _glob.glob(os.path.join(os.getcwd(), '**', filename), recursive=True)
    if cwd_matches:
        return cwd_matches[0]

    return candidate


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_gazebo')
    ugv_description_parent = os.path.dirname(get_package_share_directory('ugv_description'))
    gazebo_ros_lib = os.path.join(
        os.path.dirname(os.path.dirname(get_package_share_directory('gazebo_ros'))),
        'lib'
    )

    world = os.path.join(pkg_share, 'worlds', 'ugv_world.world')

    bird_manager_py = _find_script('bird_manager.py', pkg_share)

    print(f'[bringup] bird_manager.py -> {bird_manager_py}')

    use_gui = LaunchConfiguration('use_gui', default='true')
    enable_bird_manager = LaunchConfiguration('enable_bird_manager', default='true')
    enable_trial_logger = LaunchConfiguration('enable_trial_logger', default='false')
    trial_log_dir = LaunchConfiguration('trial_log_dir', default='~/ros2_ws/bird_patrol_data')
    trial_session_name = LaunchConfiguration('trial_session_name', default='bird_patrol_10m')

    gazebo_model_database_uri = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=(
            os.path.join(pkg_share, 'models') + ':'
            + ugv_description_parent + ':'
            + os.environ.get('GAZEBO_MODEL_PATH', '')
        )
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value=(
            os.path.join(pkg_share, 'worlds') + ':'
            + os.path.join(pkg_share, 'models') + ':'
            + '/usr/share/gazebo-11:/usr/share/gazebo:'
            + os.environ.get('GAZEBO_RESOURCE_PATH', '')
        )
    )

    gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=(
            gazebo_ros_lib + ':'
            + os.environ.get('GAZEBO_PLUGIN_PATH', '')
        )
    )

    gzserver_cmd = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            world,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen'
    )

    gzclient_cmd = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(use_gui),
    )

    run_bird_manager_cmd = TimerAction(
        period=4.0,
        condition=IfCondition(enable_bird_manager),
        actions=[
            ExecuteProcess(
                cmd=['python3', bird_manager_py],
                output='screen',
                additional_env={
                    'PYTHONUNBUFFERED': '1',
                    'BIRD_MANAGER_ACTIVE_BIRDS': 'bird_single',
                },
            )
        ]
    )

    run_trial_logger_cmd = TimerAction(
        period=7.0,
        condition=IfCondition(enable_trial_logger),
        actions=[
            Node(
                package='ugv_gazebo',
                executable='gazebo_trial_data_logger.py',
                name='gazebo_trial_data_logger',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': True,
                        'output_dir': trial_log_dir,
                        'session_name': trial_session_name,
                        'sample_period_s': 1.0,
                        'map_side_m': 15.0,
                        'patrol_side_m': 10.0,
                    }
                ],
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('use_gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_bird_manager', default_value='true'))
    ld.add_action(DeclareLaunchArgument('enable_trial_logger', default_value='false'))
    ld.add_action(DeclareLaunchArgument('trial_log_dir', default_value='~/ros2_ws/bird_patrol_data'))
    ld.add_action(DeclareLaunchArgument('trial_session_name', default_value='bird_patrol_10m'))
    ld.add_action(gazebo_model_database_uri)
    ld.add_action(gazebo_model_path)
    ld.add_action(gazebo_resource_path)
    ld.add_action(gazebo_plugin_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(run_bird_manager_cmd)
    ld.add_action(run_trial_logger_cmd)

    return ld
