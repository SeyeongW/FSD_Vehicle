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

    launch_file_dir = os.path.join(pkg_share, 'launch', 'bringup')
    world = os.path.join(pkg_share, 'worlds', 'plane_fit_world.world')
    bird_model_file = os.path.join(pkg_share, 'models', 'bird', 'model.sdf')

    bird_manager_py = _find_script('bird_manager.py', pkg_share)
    ugv_manager_py = _find_script('ugv_manager.py', pkg_share)

    print(f'[bringup] bird_manager.py -> {bird_manager_py}')
    print(f'[bringup] ugv_manager.py  -> {ugv_manager_py}')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo_model_database_uri = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models') + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
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
        output='screen'
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
        }.items()
    )

    # 새들도 전부 (0,0) 근처에 스폰
    spawn_bird_single_cmd = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'bird_single',
                    '-file', bird_model_file,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '15.0',
                ],
                output='screen'
            )
        ]
    )

    spawn_swarm_cmd = TimerAction(
        period=5.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'bird_swarm_1',
                    '-file', bird_model_file,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '20.0',
                ],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'bird_swarm_2',
                    '-file', bird_model_file,
                    '-x', '3.0',
                    '-y', '2.0',
                    '-z', '19.0',
                ],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'bird_swarm_3',
                    '-file', bird_model_file,
                    '-x', '-3.0',
                    '-y', '-2.0',
                    '-z', '21.0',
                ],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'bird_swarm_4',
                    '-file', bird_model_file,
                    '-x', '4.0',
                    '-y', '-3.0',
                    '-z', '18.0',
                ],
                output='screen'
            ),
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                    '-entity', 'bird_swarm_5',
                    '-file', bird_model_file,
                    '-x', '-4.0',
                    '-y', '3.0',
                    '-z', '22.0',
                ],
                output='screen'
            ),
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

    run_ugv_manager_cmd = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', ugv_manager_py],
                output='screen',
                additional_env={'PYTHONUNBUFFERED': '1'},
            )
        ]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_model_database_uri)
    ld.add_action(gazebo_model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_ugv_cmd)
    ld.add_action(spawn_bird_single_cmd)
    # ld.add_action(spawn_swarm_cmd)
    ld.add_action(run_bird_manager_cmd)
    ld.add_action(run_ugv_manager_cmd)

    return ld
