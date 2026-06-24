import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare launch arguments
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf', default_value='true',
        description='Whether to publish the tf from the original odom to the base_footprint'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to launch RViz2'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='bringup',
        description='Choose which rviz configuration to use'
    )
    start_driver_arg = DeclareLaunchArgument(
        'start_driver', default_value='false',
        description='Start legacy ugv_driver direct motor writer. Keep false for Waver safety-mux profiles.'
    )
    legacy_driver_enabled_arg = DeclareLaunchArgument(
        'legacy_driver_enabled', default_value='false',
        description='Explicit confirmation for legacy ugv_driver. Both this and start_driver must be true.'
    )
    start_base_feedback_arg = DeclareLaunchArgument(
        'start_base_feedback', default_value='false',
        description='Open legacy base feedback serial reader. Keep false unless the serial port is dedicated to feedback.'
    )
    feedback_serial_port_arg = DeclareLaunchArgument(
        'feedback_serial_port', default_value='',
        description='Serial port for feedback-only ugv_bringup. Leave empty for legacy probing; real profiles should use /dev/serial/by-id.'
    )
    feedback_baudrate_arg = DeclareLaunchArgument(
        'feedback_baudrate', default_value='115200',
        description='Feedback serial baudrate for ugv_bringup.'
    )
    base_node_executable_arg = DeclareLaunchArgument(
        'base_node_executable', default_value='base_node',
        description='Use base_node_ekf for /odom_raw output when robot_localization owns /odom and odom->base_link TF.'
    )
    enable_legacy_base_arg = DeclareLaunchArgument(
        'enable_legacy_ugv_base_odometry_node', default_value='true',
        description='Start legacy ugv_base_node/base_node_ekf odometry node. Set false when waver_base_driver_node owns real base feedback.'
    )
    enable_ldlidar_arg = DeclareLaunchArgument(
        'enable_ldlidar', default_value='false',
        description='Start physical 2D ldlidar scan publisher. Real profile must choose this or Mid360 scan adapter, not both.'
    )
    enable_rf2o_arg = DeclareLaunchArgument(
        'enable_rf2o', default_value='false',
        description='Start RF2O laser odometry. Real EKF profile keeps this false unless explicitly requested.'
    )

    # Include the robot state launch from the ugv_description package
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()
    )

    # Define the nodes to be launched
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        condition=IfCondition(LaunchConfiguration('start_base_feedback')),
        parameters=[
            {
                'feedback_only': True,
                'serial_port': LaunchConfiguration('feedback_serial_port'),
                'baudrate': LaunchConfiguration('feedback_baudrate'),
            }
        ],
    )

    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration('start_driver'),
                    "' == 'true' and '",
                    LaunchConfiguration('legacy_driver_enabled'),
                    "' == 'true'",
                ]
            )
        ),
    )

    optional_includes = []
    try:
        optional_includes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('ldlidar'), 'launch', 'ldlidar.launch.py')
                ),
                condition=IfCondition(LaunchConfiguration('enable_ldlidar')),
            )
        )
    except PackageNotFoundError:
        optional_includes.append(LogInfo(msg='ldlidar package not found; skipping 2D lidar bringup. Use Mid360 /scan adapter if needed.'))

    try:
        optional_includes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py')
                ),
                condition=IfCondition(LaunchConfiguration('enable_rf2o')),
            )
        )
    except PackageNotFoundError:
        optional_includes.append(LogInfo(msg='rf2o_laser_odometry package not found; skipping RF2O odom bringup.'))

    # Define the base node with parameters
    base_node = Node(
        package='ugv_base_node',
        executable=LaunchConfiguration('base_node_executable'),
        condition=IfCondition(LaunchConfiguration('enable_legacy_ugv_base_odometry_node')),
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    # Return the launch description with all defined actions
    return LaunchDescription([
        pub_odom_tf_arg,
        use_rviz_arg,
        rviz_config_arg,
        start_driver_arg,
        legacy_driver_enabled_arg,
        start_base_feedback_arg,
        feedback_serial_port_arg,
        feedback_baudrate_arg,
        base_node_executable_arg,
        enable_legacy_base_arg,
        enable_ldlidar_arg,
        enable_rf2o_arg,
        LogInfo(
            msg='bringup_lidar.launch.py: legacy serial feedback and ugv_driver default are disabled; final /cmd_vel must pass through safety mux.'
        ),
        robot_state_launch,
        bringup_node,
        driver_node,
        *optional_includes,
        base_node
    ])
