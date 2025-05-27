import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    # ─── Paths ───────────────────────────────────────────────────────────────
    share_dir = get_package_share_directory('rosbag_manager')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'bag.rviz')

    # ─── RViz Node ───────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # ─── Return Launch Description ────────────────────────────────────────────
    return LaunchDescription([
        rviz_node
    ])
