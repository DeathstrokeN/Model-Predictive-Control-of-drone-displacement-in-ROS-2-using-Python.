from glob import glob
import os
from setuptools import setup

package_name = 'drone_mpc_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 Python package for MPC-based drone displacement control.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reference_node = drone_mpc_ros.reference_node:main',
            'drone_simulator_node = drone_mpc_ros.drone_simulator_node:main',
            'mpc_controller_node = drone_mpc_ros.mpc_controller_node:main',
        ],
    },
)
