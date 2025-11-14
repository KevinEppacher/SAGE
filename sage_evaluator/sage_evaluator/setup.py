from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sage_evaluator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.py', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/**/*.yaml', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kevin-eppacher@hotmail.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Core evaluator nodes
            'evaluator_dashboard = sage_evaluator.evaluator_dashboard_node:main',
            'trajectory_recorder = sage_evaluator.trajectory_recorder_node:main',
            'dataset_writer = sage_evaluator.dataset_writer_node:main',
            # Semantic global mapping nodes
            'semantic_pcl_loader = sage_evaluator.semantic_pcl_loader_node:main',
            'pose_offset_cacher = sage_evaluator.pose_offset_cacher_node:main',
            'initial_pose_publisher = sage_evaluator.initial_pose_publisher:main',
            'nearest_target_planner = sage_evaluator.nearest_target_planner_node:main',
            'navigable_target_projector = sage_evaluator.navigable_target_projector_node:main',
            # Tests
            'test_nav2_path_action = sage_evaluator.test.test_nav2_path_action:main',
        ],
    },
)
