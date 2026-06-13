from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/mid360_PointCloud2',
            description='PointCloud2 입력 토픽 (sim/hw 모두 /mid360_PointCloud2)',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Gazebo 시뮬레이션 시간 사용 여부',
        ),
        Node(
            package='plane_fit_pkg',
            executable='plane_fit_node',
            name='plane_fit_node',
            output='screen',
            parameters=[{
                'input_topic':  LaunchConfiguration('input_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'distance_threshold': 0.05,
                'ransac_iterations':  500,
                'voxel_size':         0.05,
                'max_points':         15000,
                'process_every_n':    5,
                'z_min':              -2.0,
                'z_max':              5.0,
                'range_max':          25.0,
            }],
        ),
    ])
