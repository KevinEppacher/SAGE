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

    dm = DatasetManager(scene="00809-Qpor2mEya8F", version="v1.1")

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

    # evaluator_map_config_path = os.path.join(
    #     get_package_share_directory('sage_evaluator'),
    #     'config',
    #     'evaluator_map.yaml'
    # )

    #---------------------- Nodes ------------------------------#

    evaluator_planner_node = LifecycleNode(
        package='evaluator_planner',
        executable='evaluator_planner_node',
        name='planner_server',
        output='screen',
        emulate_tty=True,
        namespace=namespace,
        parameters=[
            {'use_sim_time': use_sim_time},
            # explorer_config
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

    delayed_lcm = TimerAction(
        period=2.0,
        actions=[lcm]
    )

    #---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(evaluator_planner_node)
    ld.add_action(delayed_lcm)
    return ld