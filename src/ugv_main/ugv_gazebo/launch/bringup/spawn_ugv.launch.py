# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile

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

    # Livox CSV 경로를 현재 환경에 맞게 동적 치환
    livox_share_dir = get_package_share_directory('ros2_livox_simulation')
    actual_csv_path = os.path.join(livox_share_dir, 'scan_mode', 'mid360.csv')

    # 원본 SDF 파일 읽기
    with open(sdf_path, 'r') as f:
        sdf_content = f.read()

    # 하드코딩된 Docker 경로를 실제 경로로 치환
    sdf_content = sdf_content.replace(
        '/ros2_ws/ugv_ws/install/ros2_livox_simulation/share/ros2_livox_simulation/scan_mode/mid360.csv',
        actual_csv_path
    )

    # 치환된 SDF를 임시 파일로 저장
    ws_path = os.environ.get('UGV_WS_PATH', '/ros2_ws/ugv_ws')
    tmp_dir = os.path.join(ws_path, '.tmp')
    os.makedirs(tmp_dir, exist_ok=True)
    tmp_sdf_path = os.path.join(tmp_dir, f'{UGV_MODEL}_model.sdf')
    with open(tmp_sdf_path, 'w') as f:
        f.write(sdf_content)

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', UGV_MODEL,
            '-file', tmp_sdf_path
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld

