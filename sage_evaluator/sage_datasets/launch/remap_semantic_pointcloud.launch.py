from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sage_datasets.utils import DatasetManager
import os

def launch_setup(context, *args, **kwargs):
    """Function to evaluate LaunchConfiguration and create nodes"""

    # ---------------------- Arguments ------------------------------ #
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    scene = LaunchConfiguration('scene').perform(context)
    version = LaunchConfiguration('version').perform(context)

    dm = DatasetManager(scene, version)

    # ---------------------- Paths ------------------------------ #
    rviz_config = os.path.join(
        get_package_share_directory('sage_datasets'),
        'rviz',
        'rviz.rviz'
    )

    # ---------------------- Nodes ------------------------------ #
    # Publishes: map → robot_original_pose_at_scan
    # (identity transform at 0,0,0 with no rotation)
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_robot_original_tf",
        arguments=[
            "0", "0", "0",     # translation (x y z)
            "0", "0", "0",     # rotation (roll pitch yaw)
            "map", "robot_original_pose_at_scan"
        ],
        output="screen"
    )

    remap_pointcloud_cluster_node = Node(
        package="sage_datasets",
        executable='remap_pointcloud_cluster',
        name="remap_semantic_pointcloud",
        output='screen',
        namespace=namespace,
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time == 'true',
                'ply_path': dm.pointcloud(),
                'json_path': dm.semantic_pcl_classes(),
            }
        ]
    )

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config],
        output='screen'
    )

    return [
        static_tf_node,
        remap_pointcloud_cluster_node,
        rviz_node
    ]


def generate_launch_description():

    console_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Flag to enable use_sim_time'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='evaluator',
        description='Namespace for evaluator Nav2 stack'
    )

    scene_arg = DeclareLaunchArgument(
        'scene',
        # default_value='00800-TEEsavR23oF',
        default_value='00809-Qpor2mEya8F',
        description='Scene ID for the evaluation'
    )

    version_arg = DeclareLaunchArgument(
        'version',
        # default_value='v1.7',
        default_value='v1.3',
        description='Version for the evaluation'
    )

    ld = LaunchDescription()
    ld.add_action(console_format)
    ld.add_action(sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(scene_arg)
    ld.add_action(version_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
