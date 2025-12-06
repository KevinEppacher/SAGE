from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import  ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    #---------------------- Arguments ------------------------------#

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    #---------------------- Paths ------------------------------#
    
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

    robot_description_launch_file = os.path.join(
        get_package_share_directory('carter_description'),
        'launch',
        'display.launch.py'
    )

    graph_node_fusion_launch_path = os.path.join(
        get_package_share_directory('graph_node_fusion'),
        'launch',
        'graph_node_fusion.launch.py'
    )

    relevance_map_launch_path = os.path.join(
        get_package_share_directory('relevance_map'),
        'launch',
        'relevance_map.launch.py'
    )

    #---------------------- Nodes ------------------------------#

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

    map_explorer_node = Node(
        package='semantic_frontier_exploration',
        executable='map_explorer_node',
        name="map_explorer_node",
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

    # graph_node_fusion_node = Node(
    #     package='graph_node_fusion',
    #     executable='graph_node_fusion',
    #     name='graph_node_fusion',
    #     namespace='fused',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[
    #         {'use_sim_time': use_sim_time},
    #         explorer_config
    #     ],
    # )

    #---------------------- Launches ------------------------------#

    nav2_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    graph_node_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(graph_node_fusion_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_path': explorer_config
        }.items()
    )

    relevance_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(relevance_map_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_path': explorer_config
        }.items()
    )

    #---------------------- Execute Processes ------------------------------#

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    #---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(inflated_map_node)
    ld.add_action(map_explorer_node)
    ld.add_action(pcl_to_scan_node)
    ld.add_action(semantic_frontiers_node)
    ld.add_action(rviz_node)
    ld.add_action(nav2_stack_launch)
    ld.add_action(robot_description_launch)
    # ld.add_action(graph_node_fusion_launch)
    ld.add_action(relevance_map_launch)
    return ld