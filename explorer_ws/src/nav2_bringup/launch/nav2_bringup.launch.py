from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    nav2_package = 'nav2_bringup'
    slam_package = 'slam_toolbox'

    # Pfade zu Konfigurationen
    nav2_share = FindPackageShare(nav2_package).find(nav2_package)
    slam_share = FindPackageShare(slam_package).find(slam_package)

    rviz_config = os.path.join(nav2_share, 'rviz', 'rviz.rviz')
    slam_launch_file = os.path.join(slam_share, 'launch', 'online_async_launch.py')
    slam_params_file = os.path.join(nav2_share, 'config', 'mapper_params_online_async.yaml')

    # RViz
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    # SLAM Toolbox
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={'slam_params_file': slam_params_file}.items()
    )

    return LaunchDescription([
        slam_toolbox_node,
        rviz_node
    ])
