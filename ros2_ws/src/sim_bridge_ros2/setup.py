import os
from glob import glob

from setuptools import setup

package_name = 'sim_bridge_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manifold Tech Ltd.',
    maintainer_email='yuhaowen@manifoldtech.cn',
    description='Isaac Sim TCP <-> ROS2 bridge',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = sim_bridge_ros2.bridge_node:main',
        ],
    },
)
