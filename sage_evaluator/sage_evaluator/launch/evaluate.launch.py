from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


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

    # detection_config = os.path.join(
    #     get_package_share_directory("sage_commander"),
    #     'config',
    #     'detection_config.yaml'
    # )

    bt_launch_path = os.path.join(
        get_package_share_directory("sage_behaviour_tree"),
        'launch',
        'sage_behavior_tree.launch.py'
    )

    bt_xml_path = os.path.join(
        get_package_share_directory("sage_behaviour_tree"),
        'bt_xml',
        'multiple_search_evaluator.xml'
    )

    sage_bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bt_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'bt_xml_path': bt_xml_path,
            'launch_groot': 'true',
        }.items()
    )


    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg) 
    ld.add_action(sage_bt_launch)
    return ld