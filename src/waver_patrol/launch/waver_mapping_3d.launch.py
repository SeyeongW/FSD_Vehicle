from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ugv_slam"),
                "launch",
                "rtabmap_rgbd.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_rtabmap_viz": LaunchConfiguration("use_rtabmap_viz"),
            "localization": LaunchConfiguration("localization"),
            "start_lidar_bringup": LaunchConfiguration("start_lidar_bringup"),
            "start_camera_bringup": LaunchConfiguration("start_camera_bringup"),
            "start_robot_pose_publisher": LaunchConfiguration("start_robot_pose_publisher"),
            "rgb_image_topic": LaunchConfiguration("rgb_image_topic"),
            "rgb_camera_info_topic": LaunchConfiguration("rgb_camera_info_topic"),
            "depth_image_topic": LaunchConfiguration("depth_image_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "frame_id": LaunchConfiguration("frame_id"),
            "queue_size": LaunchConfiguration("queue_size"),
            "qos": LaunchConfiguration("qos"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("start_lidar_bringup", default_value="true"),
            DeclareLaunchArgument("start_camera_bringup", default_value="true"),
            DeclareLaunchArgument("start_robot_pose_publisher", default_value="true"),
            DeclareLaunchArgument("rgb_image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("rgb_camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("depth_image_topic", default_value="/camera/depth/image_raw"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("frame_id", default_value="base_footprint"),
            DeclareLaunchArgument("queue_size", default_value="20"),
            DeclareLaunchArgument("qos", default_value="2"),
            rtabmap,
        ]
    )
