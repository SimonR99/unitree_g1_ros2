from glob import glob
from setuptools import setup

package_name = 'g1_ros2_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/description/urdf', glob('description/urdf/*.urdf')),
        ('share/' + package_name + '/description/meshes', glob('description/meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simonroy',
    maintainer_email='simonroy505@gmail.com',
    description='Bridge between Unitree G1 native DDS topics and standard ROS2 interfaces.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_bridge = g1_ros2_bridge.state_bridge:main',
            'odom_bridge = g1_ros2_bridge.odom_bridge:main',
            'cmd_vel_bridge = g1_ros2_bridge.cmd_vel_bridge:main',
            'loco_bridge = g1_ros2_bridge.loco_bridge:main',
            'realsense_publisher = g1_ros2_bridge.realsense_publisher:main',
        ],
    },
)
