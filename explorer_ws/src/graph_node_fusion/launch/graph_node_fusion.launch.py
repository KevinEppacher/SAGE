from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    param_file = os.path.join(
        get_package_share_directory('graph_node_fusion'), 
        'config', 
        'graph_node_fusion.yml'
    )

    graph_node = Node(
        package='graph_node_fusion',
        executable='graph_node_fusion',
        name='graph_node_fusion',
        namespace='fused',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            param_file
        ],
    )

    ld  = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(graph_node)
    return ld