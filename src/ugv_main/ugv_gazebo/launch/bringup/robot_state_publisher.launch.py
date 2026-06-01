import os

# Import the necessary modules from the ament_index_python package
from ament_index_python.packages import get_package_share_directory
# Import the necessary modules from the launch package
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Define a function to generate the launch description
def generate_launch_description():
    # Create a LaunchDescription object
    ld = LaunchDescription()
    
    # Get the UGV_MODEL environment variable
    UGV_MODEL = os.environ['UGV_MODEL']
    # Create the urdf_file_name variable by appending '.urdf' to the UGV_MODEL variable
    urdf_file_name = UGV_MODEL + '.urdf'
    # Create the urdf_model_path variable by joining the package share directory, 'urdf' folder, and the urdf_file_name
    urdf_model_path = os.path.join(
        get_package_share_directory('ugv_gazebo'),
        'urdf', 
        urdf_file_name)  
    
    # Create a LaunchConfiguration object for the use_sim_time variable
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Create a DeclareLaunchArgument object for the use_sim_time variable
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='true', description='Use simulation (Gazebo) clock if true')
    
    # Create a Node object for the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{
             'use_sim_time': use_sim_time
        }],
        )
                
    # joint_state_publisher: URDF의 모든 관절 상태를 /joint_states 로 퍼블리시.
    # source_list에 /gazebo/joint_states 를 넣으면 Gazebo가 실제로 움직이는 관절
    # (바퀴 4개 + 틸트)은 Gazebo 실측값을 쓰고, 나머지(pan 등 고정 관절)만 기본값을 채움.
    # 이렇게 하면 joint_state_publisher 가 틸트 관절을 0으로 덮어쓰는 문제를 방지.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{
             'use_sim_time': use_sim_time,
             'source_list': ['/gazebo/joint_states'],
        }],
        )
                
    # Add the use_sim_time_arg to the LaunchDescription
    ld.add_action(use_sim_time_arg)     
    # Add the robot_state_publisher_node to the LaunchDescription
    ld.add_action(robot_state_publisher_node) 
    # Add the joint_state_publisher_node to the LaunchDescription
    ld.add_action(joint_state_publisher_node)
    
    # Return the LaunchDescription
    return ld
