from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['gui', 'gui.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add templates and static files
        (os.path.join('share', package_name, 'templates'), glob('gui/templates/*.html')),
        (os.path.join('share', package_name, 'static'), glob('gui/static/*')),
    ],
    include_package_data=True,
    package_data={
        'gui': ['templates/*.html', 'static/*']
    },
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='kevin-eppacher@hotmail.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = gui.gui:main',
            'test_image = gui.test.visualize_image:main',
        ],
    },
)