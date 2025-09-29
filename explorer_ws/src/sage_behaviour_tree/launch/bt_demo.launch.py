from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('sage_behaviour_tree')
    xml = os.path.join(pkg, 'trees', 'demo.xml')
    return LaunchDescription([
        Node(
            package='sage_behaviour_tree',
            executable='bt_demo_node',
            name='bt_demo_node',
            parameters=[{'bt_xml': xml}]
        )
    ])
