import os
import glob as _glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _find_script(filename: str, pkg_share: str) -> str:
    candidate = os.path.normpath(
        os.path.join(pkg_share, '..', '..', 'lib', 'ugv_gazebo', filename)
    )
    if os.path.isfile(candidate):
        return candidate

    ws_src = os.path.expanduser('~/ros2_ws/ugv_hack/src')
    matches = _glob.glob(os.path.join(ws_src, '**', filename), recursive=True)
    if matches:
        return matches[0]

    cwd_matches = _glob.glob(os.path.join(os.getcwd(), '**', filename), recursive=True)
    if cwd_matches:
        return cwd_matches[0]

    return candidate


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_gazebo')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    launch_file_dir = os.path.join(pkg_share, 'launch', 'bringup')
    world = os.path.join(pkg_share, 'worlds', 'plane_fit_world.world')
    bird_model_file = os.path.join(pkg_share, 'models', 'bird', 'model.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'ugv_bridge.yaml')

    bird_manager_py = _find_script('bird_manager.py', pkg_share)
    print(f'[bringup] bird_manager.py -> {bird_manager_py}')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Let gz resolve `model://ugv_description/...` (meshes) and
    # `model://bird`, `model://world`, `model://<ugv_model>` (local models).
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            os.path.join(pkg_share, 'models'),
            os.path.dirname(get_package_share_directory('ugv_description')),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    # Start gz sim (server + GUI). `-r` runs immediately, `-v4` is verbose.
    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v4 {world}'}.items()
    )

    # ros <-> gz topic bridge (clock, cmd_vel, odom, tf, imu, scan, lidar, cameras).
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

    # UGV를 (0,0)에 스폰
    spawn_ugv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_ugv.launch.py')
        ),
        launch_arguments={
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': '0.1',
        }.items()
    )

    # 새 스폰 (0,0 상공)
    spawn_bird_single_cmd = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'bird_single',
                    '-file', bird_model_file,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '15.0',
                ],
                output='screen',
            )
        ]
    )

    run_bird_manager_cmd = TimerAction(
        period=8.5,
        actions=[
            ExecuteProcess(
                cmd=['python3', bird_manager_py],
                output='screen',
                additional_env={'PYTHONUNBUFFERED': '1'},
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(gz_resource_path)
    ld.add_action(gz_sim_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_ugv_cmd)
    ld.add_action(spawn_bird_single_cmd)
    ld.add_action(run_bird_manager_cmd)

    return ld
