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

    evaluator_map_config_path = os.path.join(
        get_package_share_directory('sage_evaluator'),
        'config',
        'evaluator_map.yaml'
    )

    #---------------------- Nodes ------------------------------#

    pose_offset_cacher_node = Node(
        package="sage_evaluator",
        executable='pose_offset_cacher',
        name="pose_offset_cacher",
        output='screen',
        namespace='evaluator',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'cache_path': dm.pose(),
            }
        ]
    )

    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        namespace='evaluator',
        # arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': dm.map()
            },
            evaluator_map_config_path
        ]        
    )

    semantic_pcl_loader_node = Node(
        package="sage_evaluator",
        executable='semantic_pcl_loader',
        name="semantic_pcl_loader",
        output='screen',
        namespace='evaluator',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'ply_path': dm.pointcloud(),
                'json_path': dm.semantic_pcl_classes(),
            }
        ]
    )

    global_costmap_node = LifecycleNode(
        package='evaluator_costmap',
        executable='evaluator_costmap_node',
        name='global_costmap',
        namespace='evaluator',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
            evaluator_map_config_path,
        ]
    )

    # planner_server_node = LifecycleNode(
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     output='screen',
    #     emulate_tty=True,
    #     namespace='evaluator',
    #     respawn=True,
    #     respawn_delay=2.0,
    #     parameters=[
    #         {'use_sim_time': True},
    #         explorer_config
    #     ],
    # )

    #---------------------- Launches ------------------------------#

    #---------------------- Lifecycle Manager ------------------------------#

    lifecycle_nodes = [
        '/evaluator/map_server',
        '/evaluator/global_costmap',
        # '/evaluator/planner_server',
    ]

    lcm = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        # namespace='evaluator',
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
    # delayed_initial_pose_publisher = TimerAction(
    #     period=time_const * i,
    #     actions=[initial_pose_publisher_node]
    # )

    #---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(map_server_node)
    ld.add_action(delayed_lcm)
    ld.add_action(pose_offset_cacher_node)
    ld.add_action(semantic_pcl_loader_node)
    ld.add_action(global_costmap_node)
    # ld.add_action(planner_server_node)
    return ld