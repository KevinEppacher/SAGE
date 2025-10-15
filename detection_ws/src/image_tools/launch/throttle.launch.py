from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    image_tools_config = os.path.join(
        get_package_share_directory("image_tools"),
        'config',
        'image_tools.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory("image_tools"),
        'rviz',
        'rviz.rviz'
    )

    image_tools = Node(
        package="image_tools",
        executable='image_tools_node',
        name="image_throttle",
        output='screen',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'use_sim_time': use_sim_time},
            image_tools_config
        ]
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(image_tools)
    ld.add_action(rviz_node)
    return ld
