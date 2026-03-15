from setuptools import setup
import os
from glob import glob

package_name = 'mobile_robot_edge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rx178nwj',
    maintainer_email='rx178.nwj@gmail.com',
    description='Edge-side ROS2 package for mobile robot: sensor acquisition and motor control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_publisher = mobile_robot_edge.odometry_publisher:main',
            'motor_controller = mobile_robot_edge.motor_controller:main',
            'ws_odometry_publisher = mobile_robot_edge.ws_odometry_publisher:main',
            'ws_motor_controller = mobile_robot_edge.ws_motor_controller:main',
        ],
    },
)
