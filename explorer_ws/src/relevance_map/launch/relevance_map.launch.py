from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    #---------------------- Arguments and Paths ------------------------------#

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    param_file = os.path.join(
        get_package_share_directory('relevance_map'), 
        'config', 
        'relevance_map.yaml'
    )

    config_path_arg = DeclareLaunchArgument(
        'config_path', default_value=param_file,
        description='Path to the configuration file'
    )

    config_path = LaunchConfiguration('config_path')

    #---------------------- Nodes ------------------------------#

    explorer_relevance_map_node = Node(
        package='relevance_map',
        executable='relevance_map_node',
        name='relevance_map',
        namespace='explorer_relevance_map',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            config_path
        ],
    )

    exploitation_relevance_map_node = Node(
        package='relevance_map',
        executable='relevance_map_node',
        name='relevance_map',
        namespace='exploitation_relevance_map',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            config_path
        ],
    )

    exploitation_relevance_map_node = Node(
        package='relevance_map',
        executable='relevance_map_node',
        name='relevance_map',
        namespace='detection_relevance_map',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            config_path
        ],
    )

    #---------------------- Launch Description ------------------------------#

    ld  = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(config_path_arg)
    ld.add_action(explorer_relevance_map_node)
    ld.add_action(exploitation_relevance_map_node)
    return ld