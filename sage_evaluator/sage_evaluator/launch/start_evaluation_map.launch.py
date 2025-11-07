from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

MAP_PATH = "/app/src/sage_evaluator/datasets/matterport_isaac/00809-Qpor2mEya8F/annotations/v1.1/slam_map_20251107_154330.yaml"

def generate_launch_description():

    console_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    scene_config = os.path.join(
        get_package_share_directory("sage_evaluator"),
        'config',
        '00809-Qpor2mEya8F.yaml'
    )

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

    semantic_pcl_loader_node = Node(
        package="sage_evaluator",
        executable='semantic_pcl_loader',
        name="semantic_pcl_loader",
        output='screen',
        namespace='evaluator',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'use_sim_time': use_sim_time},
            scene_config
        ]
    )

    pose_offset_cacher_node = Node(
        package="sage_evaluator",
        executable='pose_offset_cacher',
        name="pose_offset_cacher",
        output='screen',
        namespace='evaluator',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'use_sim_time': use_sim_time},
            scene_config
        ]
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_params_path,
                'map': MAP_PATH,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_params_path,
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg)
    # ld.add_action(semantic_pcl_loader_node)
    # ld.add_action(pose_offset_cacher_node)
    ld.add_action(localization_launch)
    # ld.add_action(navigation_launch)
    return ld