from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'graph_node_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/**/*.py', recursive=True)),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz', recursive=True)),
        (os.path.join('share', package_name, 'config'), glob('config/**/*.yml', recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kevin-eppacher@hotmail.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_node_fusion = graph_node_fusion.graph_node_fusion_node:main',
            'simulate_graph_nodes = graph_node_fusion.test.simulate_graph_nodes:main',
        ],
    },
)
