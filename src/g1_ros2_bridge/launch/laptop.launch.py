"""
laptop.launch.py — Visualization-side bring-up.

This is meant for a separate machine (or a second shell on the robot) that just
wants to subscribe to the topics published by `robot.launch.py` and visualize
them in RViz.

By default this launch only starts RViz: `robot.launch.py` already publishes
`/robot_description` and the full TF tree on the robot side, so you don't need
a second copy.

If for some reason you're running RViz against a robot that does NOT have
`enable_description:=true` (e.g. you ran `robot.launch.py
enable_description:=false`), pass `enable_description:=true` here to publish
the URDF + static TFs locally instead.

Args:
  use_rviz            Launch rviz2 (default: true)
  rviz_config         Path to the RViz config (default: this package's `rviz/g1.rviz`)
  enable_description  Run robot_state_publisher + static TFs here (default: false)
  urdf_path           Path to the G1 URDF (forwarded to description.launch.py)
  quiet               Filter rmw_cyclonedds discovery noise (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('g1_ros2_bridge')
    default_rviz = os.path.join(pkg_share, 'rviz', 'g1.rviz')
    default_urdf = os.path.join(pkg_share, 'description', 'urdf', 'g1_29dof.urdf')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    enable_description = LaunchConfiguration('enable_description')
    urdf_path = LaunchConfiguration('urdf_path')
    quiet = LaunchConfiguration('quiet')

    args = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz),
        DeclareLaunchArgument('enable_description', default_value='false',
            description='Run robot_state_publisher + static TFs locally '
                        '(only needed if the robot does not)'),
        DeclareLaunchArgument('urdf_path', default_value=default_urdf),
        DeclareLaunchArgument('quiet', default_value='true'),
    ]

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'description.launch.py')),
        condition=IfCondition(enable_description),
        launch_arguments={
            'urdf_path': urdf_path,
            'quiet': quiet,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
    )

    return LaunchDescription(args + [description_launch, rviz])
