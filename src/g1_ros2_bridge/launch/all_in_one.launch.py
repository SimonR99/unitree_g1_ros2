"""
all_in_one.launch.py — Run both robot-side bridges and laptop-side viz on
this machine. Convenient when developing on the G1's onboard Jetson or when
you want a single bring-up for a single-host setup.

Passes through the union of args for robot.launch.py and laptop.launch.py.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory('g1_ros2_bridge')
    robot_launch = os.path.join(pkg_share, 'launch', 'robot.launch.py')
    laptop_launch = os.path.join(pkg_share, 'launch', 'laptop.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(laptop_launch)),
    ])
