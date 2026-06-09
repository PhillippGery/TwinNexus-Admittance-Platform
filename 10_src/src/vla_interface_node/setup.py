from setuptools import find_packages, setup

package_name = 'vla_interface_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Phillipp Gery',
    maintainer_email='gery@purdue.edu',
    description='LeRobot bridge and VLA interface for the TwinNexus Admittance Platform',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_twinnexus_robot = vla_interface_node.test_twinnexus_robot:main',
        ],
    },
)
