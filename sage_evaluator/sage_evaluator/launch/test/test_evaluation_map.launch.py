from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

# MAP_PATH = "/app/src/sage_evaluator/datasets/matterport_isaac/00809-Qpor2mEya8F/annotations/v1.1/slam_map_20251107_154330.yaml"
MAP_PATH = "/app/map.yaml"


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

    nav2_navigation_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
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
            explorer_config
        ]
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

    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(nav2_navigation_launch_path),
    #     launch_arguments={
    #             'use_sim_time': use_sim_time,
    #             'params_file': nav2_params_path,
    #     }.items()
    # )

    #---------------------- Delayed Launches ------------------------------#

    i = 1
    time_const = 5.0
    delayed_initial_pose_publisher = TimerAction(
        period=time_const * i,
        actions=[initial_pose_publisher_node]
    )
    i += 1
    # delayed_navigation_launch = TimerAction(
    #     period=time_const * i,
    #     actions=[navigation_launch]
    # )

    #---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg)
    ld.add_action(pcl_to_scan_node)
    ld.add_action(localization_launch)
    # ld.add_action(delayed_initial_pose_publisher)
    return ld