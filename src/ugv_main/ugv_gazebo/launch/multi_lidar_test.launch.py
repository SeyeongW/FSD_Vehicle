import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ugv_gazebo')
    launch_file_dir = os.path.join(pkg_share, 'launch', 'bringup')
    # Runway map (ugv_world.world) copied as-is; 2 fixed LiDARs added at the runway edges.
    world = os.path.join(pkg_share, 'worlds', 'ugv_world_3lidar.world')

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
        )
    )

    # UGV_MODEL env var (set to ugv_rover) selects the robot; spawns at (0,0).
    spawn_ugv_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_ugv.launch.py')
        )
    )

    # world -> fixed_lidar poses. Must match <pose> in ugv_gazebo/worlds/ugv_world_3lidar.world.
    static_tf_lidar1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_lidar1',
        arguments=['-7', '0', '1.8', '0', '0', '0', 'world', 'lidar1'],
        parameters=[{'use_sim_time': True}],
    )
    static_tf_lidar2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_lidar2',
        arguments=['7', '0', '1.8', '3.14159', '0', '0', 'world', 'lidar2'],
        parameters=[{'use_sim_time': True}],
    )

    # Robot spawns at world origin (0,0,0); this bridges the robot's own dynamic
    # odom->base_link->livox TF chain into the fixed 'world' frame used for fusion.
    static_tf_world_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_world_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
        parameters=[{'use_sim_time': True}],
    )

    lidar_fusion_node = Node(
        package='lidar_fusion_pkg',
        executable='lidar_fusion_node',
        name='lidar_fusion_node',
        output='screen',
        parameters=[{
            'input_topics': ['lidar1_PointCloud2', 'lidar2_PointCloud2', 'mid360_PointCloud2'],
            'target_frame': 'world',
            'output_topic': 'fused_pointcloud2',
            'use_sim_time': True,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_model_database_uri)
    ld.add_action(gazebo_model_path)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_ugv_cmd)
    ld.add_action(static_tf_lidar1)
    ld.add_action(static_tf_lidar2)
    ld.add_action(static_tf_world_odom)
    ld.add_action(lidar_fusion_node)

    return ld
