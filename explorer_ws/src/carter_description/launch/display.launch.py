from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time = LaunchConfiguration(
        'use_sim_time', 
        default='true'
    )
    
    urdf_file = os.path.join(
        get_package_share_directory('carter_description'), 
        'urdf', 
        'carter.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_desc}],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    return ld