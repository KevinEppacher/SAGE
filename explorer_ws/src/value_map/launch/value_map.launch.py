from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    package_name = 'value_map'

    # Find config and rviz path
    pkg_share = FindPackageShare(package_name).find(package_name)
    param_file = os.path.join(pkg_share, 'config', 'value_map.yaml')

    rviz_config = os.path.join(pkg_share, 'rviz', 'rviz.rviz')
    # param_file = os.path.join(pkg_share, 'config', 'value_map.yml')

    value_map_node = Node(
        package=package_name,
        executable='value_map_node',
        name="value_map_node",
        output='screen',
        emulate_tty=True,
        parameters=[param_file],
        namespace='value_map',
        # arguments=['--ros-args', '--log-level', 'debug']
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        value_map_node,
        rviz_node
    ])
