from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wsg32_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required by ament: register the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phillipp Gery',
    maintainer_email='gery@purdue.edu',
    description='Minimal ROS2 driver for Weiss WSG32 gripper (GCL TCP)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This registers the ROS2 node as a runnable command:
            # ros2 run wsg32_driver wsg32_node
            'wsg32_node = wsg32_driver.wsg32_node:main',
        ],
    },
)
