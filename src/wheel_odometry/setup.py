import os
from glob import glob
from setuptools import setup

package_name = 'wheel_odometry'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # Install YAML files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Install launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS2 Python package for wheel odometry',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = wheel_odometry.wheel_odometry_node:main',  # Node executable
            'wheel_odometry_forward = wheel_odometry.wheel_odom_forward:main',  # Node executable
        ],
    },
)
