#!/usr/bin/env python3
#
# Minimal gz (Harmonic) bringup: world + bridge + robot_state_publisher + spawn.
# Uses worlds/ugv_world.world (no birds / managers).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    launch_file_dir = os.path.join(pkg_share, 'launch', 'bringup')
    world = os.path.join(pkg_share, 'worlds', 'ugv_world.world')
    bridge_config = os.path.join(pkg_share, 'config', 'ugv_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(pkg_share, 'models'),
            os.path.dirname(get_package_share_directory('ugv_description')),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v4 {world}'}.items()
    )

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ugv_gz_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True,
        }],
        output='screen',
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_ugv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_ugv.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(gz_resource_path)
    ld.add_action(gz_sim_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_ugv_cmd)

    return ld
