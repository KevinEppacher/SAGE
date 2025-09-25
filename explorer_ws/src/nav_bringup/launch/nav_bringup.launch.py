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

    rviz_config = os.path.join(
        get_package_share_directory('nav_bringup'),
        'rviz',
        'rviz.rviz'
    )


    nav2_launch_file = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    # SLAM Toolbox
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
            # 'log_level': 'debug'
        }.items()
    )

    nav2_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items()
    )

    # Nav2 (mit Timer-Verzögerung)
    delayed_nav2_stack = TimerAction(
        period=2.0,
        actions=[nav2_stack_launch]
    )

    # RViz
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg)
    ld.add_action(slam_toolbox_node)
    # ld.add_action(rviz_node)
    ld.add_action(delayed_nav2_stack)
    return ld