import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    UGV_MODEL = os.environ['UGV_MODEL']

    sdf_path = os.path.join(
        get_package_share_directory('ugv_gazebo'),
        'models',
        UGV_MODEL,
        'model.sdf'
    )

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='UGV spawn x position')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='UGV spawn y position')

    declare_z_position_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.1',
        description='UGV spawn z position')

    # Spawn the model into the running gz world via ros_gz_sim `create`.
    spawn_ugv_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', UGV_MODEL,
            '-file', sdf_path,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
        ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(declare_z_position_cmd)
    ld.add_action(spawn_ugv_cmd)

    return ld
