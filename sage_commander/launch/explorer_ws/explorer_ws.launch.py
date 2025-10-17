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
    
    rviz_config = os.path.join(
        get_package_share_directory('sage_commander'),
        'rviz',
        'explorer_ws.rviz'
    )

    explorer_config = os.path.join(
        get_package_share_directory('sage_commander'),
        'config',
        'explorer_config.yaml'
    )

    nav2_launch_file = os.path.join(
        get_package_share_directory('nav_bringup'),
        'launch',
        'nav_bringup.launch.py'
    )

    inflated_map_node = Node(
        package='semantic_frontier_exploration',
        executable='map_inflation_node',
        name="map_inflation_node",
        namespace='exploration_graph_nodes',
        output='screen',
        emulate_tty=True,
        parameters=[
            explorer_config,
            {'use_sim_time': use_sim_time}
        ],            
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

    pcl_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pcl_to_scan',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            explorer_config
        ],
        remappings=[
            ('cloud_in', '/pcl_scan'),
            ('scan', '/scan')
        ]
    )

    nav2_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(inflated_map_node)
    ld.add_action(pcl_to_scan_node)
    ld.add_action(semantic_frontiers_node)
    ld.add_action(rviz_node)
    ld.add_action(nav2_stack_launch)
    return ld