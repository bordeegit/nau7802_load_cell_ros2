from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nau7802_load_cell_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matteo Bordignon',
    maintainer_email='matteo.bordignon@polimi.it',
    description='ROS2 driver for NAU7802 load cell sensor via I2C',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'load_cell_node = nau7802_load_cell_ros2.load_cell_node:main',
        ],
    },
)
