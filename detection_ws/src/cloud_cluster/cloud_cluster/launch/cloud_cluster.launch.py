from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    cloud_cluster_config = os.path.join(
        get_package_share_directory("cloud_cluster"),
        'config',
        'cloud_cluster.yaml'
    )

    cloud_cluster_node = Node(
        package="cloud_cluster",
        executable='cloud_cluster_node',
        name="cloud_cluster_node",
        namespace="detection_graph_nodes",
        output='screen',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'use_sim_time': use_sim_time},
            cloud_cluster_config
        ]
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg) 
    ld.add_action(cloud_cluster_node)
    return ld