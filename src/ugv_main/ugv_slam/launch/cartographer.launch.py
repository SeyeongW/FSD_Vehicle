from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    # Declare launch argument for whether to launch RViz2
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                     description='Whether to launch RViz2')                              
    start_lidar_bringup_arg = DeclareLaunchArgument(
        'start_lidar_bringup',
        default_value='true',
        description='Start real LiDAR/base bringup. Set false when Gazebo already publishes /scan and TF.',
    )
    start_robot_pose_publisher_arg = DeclareLaunchArgument(
        'start_robot_pose_publisher',
        default_value='true',
        description='Start robot_pose_publisher helper. Set false when another launch already owns TF.',
    )
                                     
    # Include launch description for bringing up the lidar
    bringup_lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
         '/bringup_lidar.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
            'start_driver': 'false',
            'legacy_driver_enabled': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_lidar_bringup')),
    )
            
    # Include launch description for robot pose publisher only if that optional helper exists.
    try:
        robot_pose_share = get_package_share_directory('robot_pose_publisher')
        robot_pose_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [os.path.join(robot_pose_share, 'launch'), '/robot_pose_publisher_launch.py'])
        ,
            condition=IfCondition(LaunchConfiguration('start_robot_pose_publisher')),
        )
    except PackageNotFoundError:
        robot_pose_publisher_launch = LogInfo(
            msg='robot_pose_publisher package not found; skipping optional helper.'
        )
        
    # Include launch description for cartographer
    cartographer_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('cartographer'), 'launch'),
         '/mapping.launch.py'])
    )   
    
    # Return launch description
    return LaunchDescription([
        use_rviz_arg,
        start_lidar_bringup_arg,
        start_robot_pose_publisher_arg,
        bringup_lidar_launch,
        robot_pose_publisher_launch,
        cartographer_launch
    ])
