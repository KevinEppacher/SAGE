from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

groot2_executable = "/usr/local/bin/Groot2.AppImage"

def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    tree_path = os.path.join(
        get_package_share_directory("sage_behaviour_tree"),
        'bt_xml',
        'test',
        'test_detection_navigation.xml'
        # 'test_single_search.xml'
    )


    # Behaviour Tree node
    sage_bt_node = Node(
        package='sage_behaviour_tree',
        executable='sage_bt_node',
        name='sage_bt_node',
        namespace='sage_behaviour_tree',
        output='screen',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'tree_xml_file': tree_path},
        ] 
    )
    
    groot_gui = ExecuteProcess(
        cmd=[groot2_executable, "--nosplash", "true", "--file", tree_path]
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(sage_bt_node)
    ld.add_action(groot_gui)
    return ld