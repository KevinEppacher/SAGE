from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

MAP_PATH = "/app/src/sage_evaluator/datasets/matterport_isaac/00809-Qpor2mEya8F/annotations/v1.1/slam_map_20251107_154330.yaml"
# MAP_PATH = "/app/map.yaml"


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

    #---------------------- Paths ------------------------------#

    nav2_localization_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'localization_launch.py'
    )

    nav2_params_path = os.path.join(
        get_package_share_directory('sage_evaluator'),
        'config',
        'carter_nav2_params.yaml'
    )

    explorer_config = os.path.join(
        get_package_share_directory('sage_commander'),
        'config',
        'explorer_config.yaml'
    )

    sage_evaluator_config = os.path.join(
        get_package_share_directory('sage_evaluator'),
        'config',
        '00809-Qpor2mEya8F.yaml'
    )

    #---------------------- Nodes ------------------------------#

    pcl_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pcl_to_scan',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            explorer_config
        ],
        remappings=[
            ('cloud_in', '/pcl_scan'),
            ('scan', '/scan')
        ]
    )

    initial_pose_publisher_node = Node(
        package="sage_evaluator",
        executable='initial_pose_publisher',
        name="initial_pose_publisher",
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            sage_evaluator_config
        ]
    )

    global_costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        emulate_tty=True,
        namespace='global_costmap',
        parameters=[
            {'use_sim_time': True},
            explorer_config
        ]
    )

    planner_server_node = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        emulate_tty=True,
        namespace='',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': True},
            explorer_config
        ],
    )

    #---------------------- Launches ------------------------------#

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_params_path,
                'map': MAP_PATH,
        }.items()
    )

    #---------------------- Lifecycle Manager ------------------------------#

    lifecycle_nodes = [
        'costmap/costmap',
        'planner_server'
    ]

    lcm = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_local_costmap',
        output='screen',
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
    delayed_initial_pose_publisher = TimerAction(
        period=time_const * i,
        actions=[initial_pose_publisher_node]
    )

    #---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg)
    ld.add_action(pcl_to_scan_node)
    ld.add_action(localization_launch)
    ld.add_action(delayed_initial_pose_publisher)
    ld.add_action(global_costmap_node)
    ld.add_action(planner_server_node)
    ld.add_action(delayed_lcm)
    return ld