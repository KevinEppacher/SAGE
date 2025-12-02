from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from sage_datasets.utils import DatasetManager
import os

SCENE = "00800-TEEsavR23oF"
VERSION = "v1.10"
PROMPT_SET = "train"
EPISODE_ID = "006"

def launch_setup(context, *args, **kwargs):
    """Function to evaluate LaunchConfiguration and create nodes"""
    
    # Get the evaluated values
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    scene = LaunchConfiguration('scene').perform(context)
    version = LaunchConfiguration('version').perform(context)
    episode_id = LaunchConfiguration('episode_id').perform(context)
    prompt_set = LaunchConfiguration('prompt_set').perform(context)

    #---------------------- Paths ------------------------------#

    evaluator_config_path = os.path.join(
        get_package_share_directory('sage_evaluator'),
        'config',
        'evaluator_map.yaml'
    )

    #---------------------- Nodes ------------------------------#

    evaluate_node = Node(
        package="sage_evaluator",
        executable='evaluator_dashboard',
        name="evaluator_dashboard",
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time == 'true',
                'scene': scene,
                'version': version,
                'prompt_set': prompt_set,
                'episode_id': episode_id
            },
            evaluator_config_path
        ]
    )

    # trajectory_recorder_node = Node(
    #     package="sage_evaluator",
    #     executable='trajectory_recorder',
    #     name="trajectory_recorder",
    #     namespace=namespace,
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[
    #         {
    #             'use_sim_time': use_sim_time == 'true',
    #         },
    #         evaluator_config_path
    #     ]
    # )

    # dataset_writer_node = Node(
    #     package="sage_evaluator",
    #     executable='dataset_writer',
    #     name="dataset_writer",
    #     namespace=namespace,
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[
    #         {
    #             'use_sim_time': use_sim_time == 'true',
    #         },
    #         evaluator_config_path
    #     ]
    # )

    #---------------------- Launch Files ------------------------------#

    start_evaluation_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('sage_evaluator'),
            'launch',
            'start_evaluation_map.launch.py'
        )),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'scene': scene,
            'version': version
        }.items()
    )

    #----------------------Delayed Nodes & Launch Files ------------------------------#

    delayed_evaluate_node = TimerAction(
        period=10.0,
        actions=[evaluate_node]
    )
    
    return [
        delayed_evaluate_node,
        start_evaluation_map_launch,
        # trajectory_recorder_node,
        # dataset_writer_node
    ]

def generate_launch_description():

    #---------------------- Arguments ------------------------------#

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
        default_value=SCENE,
        description='Scene ID for the evaluation'
    )

    version_arg = DeclareLaunchArgument(
        'version',
        default_value=VERSION,
        description='Version for the evaluation'
    )

    prompt_set_arg = DeclareLaunchArgument(
        'prompt_set',
        default_value=PROMPT_SET,
        description='Prompt set for the evaluation'
    )

    episode_id_arg = DeclareLaunchArgument(
        'episode_id',
        default_value=EPISODE_ID,
        description='Episode ID for the evaluation'
    )

    #---------------------- Launch Description ------------------------------#

    ld = LaunchDescription()
    # Output only the message to console
    ld.add_action(console_format)
    # Arguments
    ld.add_action(sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(scene_arg)
    ld.add_action(version_arg)
    ld.add_action(prompt_set_arg)
    ld.add_action(episode_id_arg)
    # Use OpaqueFunction to defer evaluation
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld