from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    nav2_package = 'nav_bringup'
    slam_package = 'slam_toolbox'

    nav2_share = FindPackageShare(nav2_package).find(nav2_package)
    slam_share = FindPackageShare(slam_package).find(slam_package)

    rviz_config = os.path.join(nav2_share, 'rviz', 'rviz.rviz')
    slam_launch_file = os.path.join(slam_share, 'launch', 'online_async_launch.py')
    slam_params_file = os.path.join(nav2_share, 'config', 'mapper_params_online_async.yaml')

    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    nav2_launch_file = os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')

    # RViz
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    # Nav2 (mit Timer-Verzögerung)
    nav2 = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch_file),
                launch_arguments={
                    'use_sim_time': 'true',
                    'autostart': 'true',
                }.items()
            )
        ]
    )

    # SLAM Toolbox
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    return LaunchDescription([
        slam_toolbox_node,
        nav2,
        rviz_node
    ])
