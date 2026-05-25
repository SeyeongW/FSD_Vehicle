from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rtabmap_viz = LaunchConfiguration("use_rtabmap_viz")
    queue_size = LaunchConfiguration("queue_size")
    qos = LaunchConfiguration("qos")
    localization = LaunchConfiguration("localization")
    start_lidar_bringup = LaunchConfiguration("start_lidar_bringup")
    start_camera_bringup = LaunchConfiguration("start_camera_bringup")
    start_robot_pose_publisher = LaunchConfiguration("start_robot_pose_publisher")
    frame_id = LaunchConfiguration("frame_id")
    rgb_image_topic = LaunchConfiguration("rgb_image_topic")
    rgb_camera_info_topic = LaunchConfiguration("rgb_camera_info_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")

    parameters = {
        "frame_id": frame_id,
        "use_sim_time": use_sim_time,
        "queue_size": ParameterValue(queue_size, value_type=int),
        "subscribe_rgb": True,
        "subscribe_depth": True,
        "subscribe_scan": True,
        "subscribe_odom_info": False,
        "approx_sync": True,
        "qos": ParameterValue(qos, value_type=int),
        "qos_image": ParameterValue(qos, value_type=int),
        "qos_camera_info": ParameterValue(qos, value_type=int),
        "Reg/Force3DoF": "true",
        "Optimizer/GravitySigma": "0",
        "Rtabmap/DetectionRate": "2.0",
    }

    remappings = [
        ("rgb/image", rgb_image_topic),
        ("rgb/camera_info", rgb_camera_info_topic),
        ("depth/image", depth_image_topic),
        ("scan", scan_topic),
        ("odom", odom_topic),
    ]

    bringup_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ugv_bringup"),
                "launch",
                "bringup_lidar.launch.py",
            )
        ),
        launch_arguments={
            "use_rviz": "false",
            "start_driver": "false",
            "legacy_driver_enabled": "false",
        }.items(),
        condition=IfCondition(start_lidar_bringup),
    )

    bringup_oak_lite_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ugv_vision"),
                "launch",
                "oak_d_lite.launch.py",
            )
        ),
        condition=IfCondition(start_camera_bringup),
    )

    rtabmap_slam_node = Node(
        condition=UnlessCondition(localization),
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap_rgbd_slam",
        output="screen",
        parameters=[parameters],
        remappings=remappings,
        arguments=["-d"],
    )

    rtabmap_localization_node = Node(
        condition=IfCondition(localization),
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap_rgbd_localization",
        output="screen",
        parameters=[
            parameters,
            {
                "Mem/IncrementalMemory": "False",
                "Mem/InitWMWithAllNodes": "True",
            },
        ],
        remappings=remappings,
    )

    rviz_config = os.path.join(
        get_package_share_directory("ugv_gazebo"),
        "rviz",
        "view_slam_3d.rviz",
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    rtabmap_viz_node = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        output="screen",
        parameters=[parameters],
        remappings=remappings,
        condition=IfCondition(use_rtabmap_viz),
    )

    robot_pose_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_pose_publisher"),
                "launch",
                "robot_pose_publisher_launch.py",
            )
        ),
        condition=IfCondition(start_robot_pose_publisher),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("queue_size", default_value="20"),
            DeclareLaunchArgument("qos", default_value="2"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("start_lidar_bringup", default_value="true"),
            DeclareLaunchArgument("start_camera_bringup", default_value="true"),
            DeclareLaunchArgument("start_robot_pose_publisher", default_value="true"),
            DeclareLaunchArgument("frame_id", default_value="base_footprint"),
            DeclareLaunchArgument("rgb_image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("rgb_camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("depth_image_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            bringup_lidar_launch,
            bringup_oak_lite_launch,
            robot_pose_publisher_launch,
            rtabmap_slam_node,
            rtabmap_localization_node,
            rviz2_node,
            rtabmap_viz_node,
        ]
    )
