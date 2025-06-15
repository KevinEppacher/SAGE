from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'value_map'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
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
            'value_map_node = value_map.value_map_node:main',   
            'test_service = value_map.test.test_service:main',      
            'add_two_ints_server = value_map.test.add_two_ints_server:main',
            'add_two_ints_client = value_map.test.add_two_ints_client:main',       
            'test_pcl = value_map.test.test_pcl:main',
        ],
    },
)