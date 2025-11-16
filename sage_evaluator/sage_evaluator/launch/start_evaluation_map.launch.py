from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sage_datasets.utils import DatasetManager
import os

def launch_setup(context, *args, **kwargs):
    """Function to evaluate LaunchConfiguration and create nodes"""
    
    # Get the evaluated values
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    scene = LaunchConfiguration('scene').perform(context)
    version = LaunchConfiguration('version').perform(context)
    
    # Now instantiate DatasetManager with actual string values
    dm = DatasetManager(scene, version)

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
        namespace=namespace,
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time == 'true',
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
        namespace=namespace,
        parameters=[
            {
                'use_sim_time': use_sim_time == 'true',
                'yaml_filename': dm.map()
                # 'yaml_filename': "/app/map.yaml"
            },
            evaluator_map_config_path
        ]        
    )

    semantic_pcl_loader_node = Node(
        package="sage_evaluator",
        executable='semantic_pcl_loader',
        name="semantic_pcl_loader",
        output='screen',
        namespace=namespace,
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time == 'true',
                'ply_path': dm.pointcloud(),
                'json_path': dm.semantic_pcl_classes(),
            }
        ]
    )

    navigable_target_projector_node = Node(
        package="sage_evaluator",
        executable='navigable_target_projector',
        name="navigable_target_projector",
        output='screen',
        namespace=namespace,
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time == 'true',
            },
            evaluator_map_config_path
        ]
    )

    nearest_target_planner = Node(
        package="sage_evaluator",
        executable='nearest_target_planner',
        name="nearest_target_planner",
        output='screen',
        namespace=namespace,
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time == 'true',
            },
            evaluator_map_config_path
        ]
    )

    shortest_path_service_node = Node(
        package="sage_evaluator",
        executable='shortest_path_service',
        name="shortest_path_service",
        output='screen',
        namespace=namespace,
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time == 'true',
            },
            evaluator_map_config_path
        ]
    )

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
            {'use_sim_time': use_sim_time == 'true'},
            evaluator_map_config_path
        ],
    )

    #---------------------- Lifecycle Manager ------------------------------#

    lifecycle_nodes = [
        '/evaluator/map_server',
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
        emulate_tty=True
    )

    #---------------------- Delayed Launches ------------------------------#

    delayed_lcm = TimerAction(
        period=2.0,
        actions=[lcm]
    )
    
    return [
        map_server_node,
        pose_offset_cacher_node,
        semantic_pcl_loader_node,
        planner_server_node,
        navigable_target_projector_node,
        nearest_target_planner,
        delayed_lcm,
        shortest_path_service_node
    ]

def generate_launch_description():

    #---------------------- Arguments ------------------------------#

    console_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='evaluator',
        description='Namespace for evaluator Nav2 stack'
    )

    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='00800-TEEsavR23oF',
        description='Scene ID for the evaluation'
    )

    version_arg = DeclareLaunchArgument(
        'version',
        default_value='v1.7',
        description='Version for the evaluation'
    )
    
    #---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    # Output only the message to console
    ld.add_action(console_format)
    # Arguments
    ld.add_action(sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(scene_arg)
    ld.add_action(version_arg)
    # Use OpaqueFunction to defer evaluation
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld