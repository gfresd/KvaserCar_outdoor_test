from setuptools import find_packages, setup

package_name = 'control_rover'

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
    maintainer='tk22',
    maintainer_email='kaiget@kth.se',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'low_level_control = control_rover.low_level_control:main',
            'low_level_control_feedback = control_rover.low_level_control_feedback:main',
            'low_level_control_easy_play = control_rover.low_level_control_easy_play:main',
            'simple_ctrl = control_rover.simple_ctrl:main'
        ],
    },
)
