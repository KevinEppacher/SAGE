import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_stylesheet = os.path.join(
        get_package_share_directory('sage_commander'),
        'rviz',
        'dark.qss'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--stylesheet', rviz_stylesheet],
            output='screen'
        )
    ])