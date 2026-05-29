#!/usr/bin/env python3

import os
import glob as _glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction


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
    world = os.path.join(pkg_share, 'worlds', 'ugv_world.world')
    bird_manager_py = _find_script('bird_manager.py', pkg_share)

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
    )

    run_bird_manager_cmd = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', bird_manager_py],
                output='screen',
                additional_env={
                    'PYTHONUNBUFFERED': '1',
                    'BIRD_MANAGER_ACTIVE_BIRDS': 'bird_single',
                    'BIRD_MANAGER_Z_MIN_M': '3.1',
                    'BIRD_MANAGER_Z_MAX_M': '3.4',
                    'BIRD_MANAGER_MIN_SPEED_MPS': '0.05',
                    'BIRD_MANAGER_MAX_SPEED_MPS': '0.18',
                },
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_model_database_uri)
    ld.add_action(gazebo_model_path)
    ld.add_action(gazebo_resource_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(run_bird_manager_cmd)

    return ld
