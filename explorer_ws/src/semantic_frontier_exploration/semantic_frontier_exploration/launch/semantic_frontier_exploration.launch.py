from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    rviz_config = os.path.join(
        get_package_share_directory('semantic_frontier_exploration'),
        'rviz',
        'rviz.rviz'
    )


    explorer_config = os.path.join(
        get_package_share_directory('semantic_frontier_exploration'),
        'config',
        'sem_frontiers.yaml'
    )

    semantic_frontiers_node = Node(
        package='semantic_frontier_exploration',
        executable='semantic_frontier_exploration_node',
        name="semantic_frontier_exploration_node",
        namespace='exploration_graph_nodes',
        output='screen',
        emulate_tty=True,
        parameters=[
            explorer_config,
            {'use_sim_time': use_sim_time}
        ],            
    )


    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(semantic_frontiers_node)
    ld.add_action(rviz_node)
    return ld