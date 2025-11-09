from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    default_params = os.path.join(
        get_package_share_directory("sage_evaluator"),
        'config', 
        'global_costmap.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )

    print("Using costmap params file: " + default_params)

    # Define LifecycleNode for the global costmap
    global_costmap_node = LifecycleNode(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        namespace='global_costmap',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # 'map_topic': '/evaluator/map',
                # 'static_layer.map_topic': '/evaluator/map'
            },
            default_params,
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        global_costmap_node
    ])
