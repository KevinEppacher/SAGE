import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Retrieve launch argument at runtime
    world_name = LaunchConfiguration('world_name').perform(context)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # Construct output path under src
    share_dir = get_package_share_directory('rosbag_manager')
    src_dir = os.path.abspath(os.path.join(share_dir, '../../../../src/rosbag_manager'))
    bag_dir = os.path.join(src_dir, 'bag', f'bag_{timestamp}_{world_name}')

    # Create ExecuteProcess action
    record_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_dir,
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '--duration', '20s',
            # '/back_2d_lidar/scan',
            # '/back_stereo_imu/imu',
            '/camera_info',
            # '/chassis/imu',
            # '/clicked_point',
            '/clock',
            # '/cmd_vel',
            '/depth',
            '/depth_pcl',
            # '/front_3d_lidar/lidar_points',
            # '/goal_pose',
            # '/initialpose',
            '/map',
            '/map_metadata',
            '/map_updates',
            '/odom',
            '/parameter_events',
            '/pose',
            '/rgb',
            '/rosout',
            # '/scan',
            # '/slam_toolbox/feedback',
            # '/slam_toolbox/graph_visualization',
            # '/slam_toolbox/scan_visualization',
            # '/slam_toolbox/update',
            '/tf',
            '/tf_static',
        ],
        output='screen'
    )

    return [record_cmd]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world_name',
            default_value='default_world',
            description='Name of the world/scenario for bag naming'
        ),
        OpaqueFunction(function=launch_setup)
    ])
