from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sage_datasets.utils import DatasetManager
import os

def generate_launch_description():

    #---------------------- Arguments ------------------------------#

    console_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='evaluator',
        description='Namespace for evaluator Nav2 stack'
    )

    namespace = LaunchConfiguration('namespace')

    #---------------------- Paths ------------------------------#

    evaluator_map_config_path = os.path.join(
        get_package_share_directory('sage_evaluator'),
        'config',
        'evaluator_map.yaml'
    )

    #---------------------- Nodes ------------------------------#

    planner_server_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        emulate_tty=True,
        namespace=namespace,
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': True},
            evaluator_map_config_path
        ],
    )

    #---------------------- Lifecycle Manager ------------------------------#

    lifecycle_nodes = [
        '/evaluator/planner_server',
    ]

    lcm = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        namespace=namespace,
        parameters=[{
            'autostart': True,
            'node_names': lifecycle_nodes,
            'bond_timeout': 0.0,
            'bond_respawn': True
        }],
        # arguments=['--ros-args', '--log-level', 'debug'],
        emulate_tty=True
    )

    #---------------------- Delayed Launches ------------------------------#

    i = 1
    time_const = 2.0
    delayed_lcm = TimerAction(
        period=time_const * i,
        actions=[lcm]
    )
    i += 1

    #---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(planner_server_node)
    ld.add_action(delayed_lcm)
    return ld