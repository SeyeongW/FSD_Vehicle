import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Function to generate launch description
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
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock for Gazebo mapping.',
    )
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LaserScan topic used by slam_gmapping.',
    )
                                     
    # Include launch description for bringup_lidar.launch.py
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
    
    # Launch slam_gmapping directly so the scan input can be remapped when needed.
    gmapping_launch = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[('scan', LaunchConfiguration('scan_topic'))],
    )

    # Include launch description for robot_pose_publisher_launch.py only if that optional helper exists.
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
        
    # Return launch description
    return LaunchDescription([
        use_rviz_arg,
        start_lidar_bringup_arg,
        start_robot_pose_publisher_arg,
        use_sim_time_arg,
        scan_topic_arg,
        bringup_lidar_launch, 
        robot_pose_publisher_launch,
        gmapping_launch
    ])
