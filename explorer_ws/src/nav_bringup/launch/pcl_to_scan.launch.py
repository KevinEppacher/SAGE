from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
import os

# source install/setup.bash && ros2 launch slam_toolbox online_async_launch.py params_file:=/app/src/nav_bringup/config/mapper_params_online_async.yaml use_sim_time:=true

def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    pcl_to_scan_config = os.path.join(
        get_package_share_directory('nav_bringup'),
        'config',
        'pcl_to_scan.yaml'
    )

    pcl_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pcl_to_scan',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            pcl_to_scan_config
        ],
        remappings=[
            ('cloud_in', '/pcl_scan'),
            ('scan', '/scan')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(pcl_to_scan_node)

    return ld