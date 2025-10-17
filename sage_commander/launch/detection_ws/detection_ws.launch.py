from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    # Get the launch configuration for use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    detection_config = os.path.join(
        get_package_share_directory("sage_commander"),
        'config',
        'detection_config.yaml'
    )

    # YOLOE lifecycle node
    yoloe_node = LifecycleNode(
        package='yolo_ros',
        executable='yolo_ros',
        name='yolo_wrapper',
        namespace='yoloe',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            detection_config
        ]
    )

    # Semantic pointcloud node
    semantic_pcl_node = Node(
        package='yolo_ros',
        executable='yolo_semantic_pointcloud',
        name='semantic_pointcloud',
        namespace='yoloe',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            detection_config
        ]
    )

    cloud_cluster_node = Node(
        package="cloud_cluster",
        executable='cloud_cluster_node',
        name="cloud_cluster_node",
        namespace="detection_graph_nodes",
        output='screen',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'use_sim_time': use_sim_time},
            detection_config
        ]
    )

    value_map_node = Node(
        package="value_map",
        executable='value_map_node',
        name="value_map",
        namespace="value_map",
        output='screen',
        emulate_tty=True,
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'use_sim_time': use_sim_time},
            detection_config
        ]
    )

    lcm = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_detection',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                # '/yoloe/yolo_wrapper',
                '/value_map/value_map'
            ]
        }]
    )

    # controller_node = Node(
    #     package='sage_commander',
    #     executable='sequential_lifecycle_launch',
    #     name='sequential_lifecycle_launch',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[detection_config]
    # )

    # start_controller = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=seem_node,
    #         on_start=[TimerAction(period=0.5, actions=[controller_node])]
    #     )
    # )

    ld = LaunchDescription()
    ld.add_action(sim_time_arg) 
    # ld.add_action(yoloe_node)
    # ld.add_action(semantic_pcl_node)
    # ld.add_action(cloud_cluster_node)
    ld.add_action(value_map_node)
    ld.add_action(lcm)
    return ld