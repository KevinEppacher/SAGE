from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
import os

groot2_executable = "/usr/local/bin/Groot2.AppImage"

def generate_launch_description():

    console_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )

    sage_bt_config = os.path.join(
        get_package_share_directory("sage_behaviour_tree"),
        'config',
        'sage_bt_config.yaml'
    )

    bt_default = os.path.join(
        get_package_share_directory("sage_behaviour_tree"),
        'bt_xml',
        # 'test',
        # 'test_seekout_graph_nodes.xml'
        'multiple_search_evaluator.xml'
    )

    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml_path', default_value=bt_default,
        description='Behavior Tree XML file path'
    )

    launch_groot_arg = DeclareLaunchArgument(
        'launch_groot', default_value='false',
        description='Launch Groot2 GUI'
    )

    # LaunchConfigurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    bt_xml_path = LaunchConfiguration('bt_xml_path')
    launch_groot = LaunchConfiguration('launch_groot')

    # Behaviour Tree node
    sage_bt_node = Node(
        package='sage_behaviour_tree',
        executable='sage_bt_node',
        name='sage_bt_node',
        namespace='sage_behaviour_tree',
        # prefix=['gdbserver :3000'],  # no localhost
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'tree_xml_file': bt_xml_path},
            sage_bt_config
        ]
    )

    groot_gui = ExecuteProcess(
        cmd=[groot2_executable, "--nosplash", "true", "--file", bt_xml_path],
        condition=IfCondition(launch_groot)
    )

    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg)
    ld.add_action(bt_xml_arg)
    ld.add_action(launch_groot_arg)
    ld.add_action(sage_bt_node)
    ld.add_action(groot_gui)
    return ld