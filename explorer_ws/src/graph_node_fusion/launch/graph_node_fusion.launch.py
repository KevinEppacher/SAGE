from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    graph_node_fusion_name = "graph_node_fusion"
    pkg_share = get_package_share_directory(graph_node_fusion_name)
    param_file = os.path.join(pkg_share, 'config', 'graph_node_fusion.yml')
    graph_namespace = 'graph_node_fusion'
    graph_node = Node(
        package='graph_node_fusion',
        executable='graph_node_fusion',
        name=graph_node_fusion_name,
        namespace=graph_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[param_file],
    )

    return LaunchDescription([
        graph_node
    ])
