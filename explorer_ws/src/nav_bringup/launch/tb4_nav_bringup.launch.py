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
        'use_sim_time', default_value='false',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_launch_file = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    slam_params_file = os.path.join(
        get_package_share_directory('nav_bringup'),
        'config',
        'mapper_params_online_async.yaml'
    )

    nav2_launch_file = os.path.join(
        get_package_share_directory('turtlebot4_navigation'),
        'launch',
        'nav2.launch.py'
    )

    # SLAM Toolbox
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    nav2_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_respawn': 'true'
        }.items()
    )
    
    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(slam_toolbox_node)
    # ld.add_action(rviz_node)
    ld.add_action(nav2_stack_launch)
    return ld