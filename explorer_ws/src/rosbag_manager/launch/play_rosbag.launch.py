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

    # ─── Launch Arguments ─────────────────────────────────────────────────────
    bag_path_arg = DeclareLaunchArgument(
        name='bag_path',
        description='Path to the rosbag directory (not the .db3 file)'
    )

    rate_arg = DeclareLaunchArgument(
        name='rate',
        default_value='1.0',
        description='Playback speed'
    )

    loop_arg = DeclareLaunchArgument(
        name='loop',
        default_value='false',
        description='Repeat playback'
    )

    # ─── Launch Setup Function ────────────────────────────────────────────────
    def launch_setup(context, *args, **kwargs):
        bag_path = LaunchConfiguration('bag_path').perform(context)
        rate = LaunchConfiguration('rate').perform(context)
        loop = LaunchConfiguration('loop').perform(context).lower() == 'true'

        # ros2 bag play command
        bag_cmd = [
            'ros2', 'bag', 'play',
            bag_path,
            '--rate', rate
        ]
        if loop:
            bag_cmd.append('--loop')

        # RViz Node
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )

        return [
            ExecuteProcess(cmd=bag_cmd, output='screen'),
            rviz_node
        ]

    # ─── Return Launch Description ────────────────────────────────────────────
    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        loop_arg,
        OpaqueFunction(function=launch_setup)
    ])
