from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
import os

def generate_launch_description():
    package_name = 'semantic_frontier_exploration'
    pkg_share = FindPackageShare(package_name).find(package_name)

    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz.rviz')
    param_file = os.path.join(pkg_share, 'config', 'sem_frontiers.yml')

    semantic_frontiers_node = Node(
        package=package_name,
        executable='semantic_frontier_exploration_node',
        name="semantic_frontier_exploration_node",
        output='screen',
        emulate_tty=True,
        parameters=[param_file]
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        PushRosNamespace('semantic_frontier_exploration'),
        semantic_frontiers_node,
        rviz_node
    ])
