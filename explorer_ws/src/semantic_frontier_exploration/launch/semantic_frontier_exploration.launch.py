from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    package_name = 'semantic_frontier_exploration'
    pkg_share = FindPackageShare(package_name).find(package_name)

    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz.rviz')
    param_file = os.path.join(pkg_share, 'config', 'sem_frontiers.yml')
    costmap_param_file = os.path.join(pkg_share, 'config', 'semantic_costmap.yaml')

    semantic_frontiers_node = Node(
        package=package_name,
        executable='semantic_frontier_exploration_node',
        name="semantic_frontier_exploration_node",
        namespace="semantic_frontier_exploration",
        output='screen',
        emulate_tty=True,
        parameters=[param_file]
    )

    semantic_costmap_node = Node(
        package='semantic_frontier_exploration',
        executable='costmap_publisher_node',
        name='semantic_costmap',
        namespace='semantic_frontier_exploration',
        parameters=[costmap_param_file],
        arguments=['--ros-args', '--log-level', 'debug'],
        output='screen',
        emulate_tty=True,
    )


    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        # semantic_frontiers_node,
        semantic_costmap_node,
        rviz_node
    ])
