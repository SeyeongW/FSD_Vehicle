"""
plane_fit_sim.launch.py
평면 피팅 데모용 단독 시뮬레이션 런치 파일.

실행 순서:
  1. Gazebo (plane_fit_world.world)
  2. robot_state_publisher + joint_state_publisher  (ugv_gazebo 재사용)
  3. UGV 스폰 (LiDAR 탑재)
  4. plane_fit_node

사전 조건:
  export UGV_MODEL=ugv_rover
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_pkg  = get_package_share_directory('ugv_gazebo')
    plane_pkg   = get_package_share_directory('plane_fit_pkg')
    bringup_dir = os.path.join(gazebo_pkg, 'launch', 'bringup')

    world_file = os.path.join(gazebo_pkg, 'worlds', 'plane_fit_world.world')

    # ---------- 런치 인자 ----------
    declare_input_topic = DeclareLaunchArgument(
        'input_topic', default_value='/mid360_PointCloud2',
        description='PointCloud2 토픽 (sim/hw 모두 /mid360_PointCloud2)')

    # ---------- Gazebo ----------
    gazebo_model_database_uri = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI', value='')
    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(gazebo_pkg, 'models') + ':'
              + os.environ.get('GAZEBO_MODEL_PATH', ''))

    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen')

    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    # ---------- Robot State Publisher (ugv_gazebo 것 재사용) ----------
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    # ---------- UGV 스폰 (3초 후: Gazebo 준비 대기) ----------
    spawn_ugv = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', os.environ.get('UGV_MODEL', 'ugv_rover'),
                '-file', os.path.join(
                    get_package_share_directory('ugv_gazebo'),
                    'models', os.environ.get('UGV_MODEL', 'ugv_rover'), 'model.sdf'),
                '-x', '0.0', '-y', '0.0', '-z', '0.05',
            ],
            output='screen')])

    # ---------- plane_fit_node (5초 후: 스폰 완료 대기) ----------
    plane_fit_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='plane_fit_pkg',
            executable='plane_fit_node',
            name='plane_fit_node',
            output='screen',
            parameters=[{
                'input_topic':  LaunchConfiguration('input_topic'),
                'use_sim_time': True,
                'distance_threshold': 0.05,
                'ransac_iterations':  500,
                'voxel_size':         0.05,
                'max_points':         15000,
                'process_every_n':    5,
                'z_min':              -2.0,
                'z_max':              5.0,
                'range_max':          25.0,
            }])])

    ld = LaunchDescription()
    ld.add_action(declare_input_topic)
    ld.add_action(gazebo_model_database_uri)
    ld.add_action(gazebo_model_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_ugv)
    ld.add_action(plane_fit_node)
    return ld
