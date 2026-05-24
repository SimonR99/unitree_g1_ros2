"""
robot.launch.py — Bring-up of the G1-side bridges.

Run this on the robot itself (or on any machine on the same DDS network).
Publishes /joint_states, /imu/data, /odom, /tf, /tf_static, /robot_description
and subscribes /cmd_vel + /g1/enable.

By default this launch ALSO runs `description.launch.py` (robot_state_publisher
plus the static TFs not in the URDF), so the robot is the single source of
truth for the TF tree. RViz on a laptop will then see a complete chain
(`odom -> base_link -> pelvis -> ... -> livox_frame / camera_*`) without
needing any local URDF.

Args:
  interface           Network interface for Unitree DDS (default: $G1_INTERFACE)
  domain_id           ROS_DOMAIN_ID for Unitree DDS (default: 0)
  enable_odom         Run the SportModeState->/odom bridge (default: true)
  enable_cmd_vel      Run the /cmd_vel -> LocoClient bridge (default: true)
  enable_loco         Run the FSM / standing service bridge (default: true)
  enable_description  Run robot_state_publisher + static TFs (default: true)
  require_enable      Require /g1/enable=true before cmd_vel passes through (default: true)
  enable_realsense    Run the on-board RealSense publisher (default: false)
  dry_run             cmd_vel_bridge logs but does not send to the robot (default: false)
  urdf_path           Path to the G1 URDF (forwarded to description.launch.py;
                      defaults to the bundled `g1_29dof.urdf`)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    interface = LaunchConfiguration('interface')
    domain_id = LaunchConfiguration('domain_id')
    enable_odom = LaunchConfiguration('enable_odom')
    enable_cmd_vel = LaunchConfiguration('enable_cmd_vel')
    enable_loco = LaunchConfiguration('enable_loco')
    enable_description = LaunchConfiguration('enable_description')
    require_enable = LaunchConfiguration('require_enable')
    enable_realsense = LaunchConfiguration('enable_realsense')
    dry_run = LaunchConfiguration('dry_run')
    urdf_path = LaunchConfiguration('urdf_path')

    args = [
        DeclareLaunchArgument('interface',
            default_value=EnvironmentVariable('G1_INTERFACE', default_value=''),
            description='Network interface for Unitree DDS (e.g. eth0, eno2)'),
        DeclareLaunchArgument('domain_id', default_value='0'),
        DeclareLaunchArgument('enable_odom', default_value='true'),
        DeclareLaunchArgument('enable_cmd_vel', default_value='true'),
        DeclareLaunchArgument('enable_loco', default_value='true',
            description='Run the FSM / standing-mode service bridge'),
        DeclareLaunchArgument('enable_description', default_value='true',
            description='Run robot_state_publisher + static TFs on this host'),
        DeclareLaunchArgument('require_enable', default_value='true'),
        DeclareLaunchArgument('enable_realsense', default_value='false'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('urdf_path',
            default_value=os.path.join(
                get_package_share_directory('g1_ros2_bridge'),
                'description', 'urdf', 'g1_29dof.urdf')),
    ]

    state = Node(
        package='g1_ros2_bridge',
        executable='state_bridge',
        name='g1_state_bridge',
        output='screen',
        parameters=[{
            'interface': interface,
            'domain_id': ParameterValue(domain_id, value_type=int),
            'publish_imu_tf': True,
        }],
    )

    odom = Node(
        package='g1_ros2_bridge',
        executable='odom_bridge',
        name='g1_odom_bridge',
        output='screen',
        condition=IfCondition(enable_odom),
        parameters=[{
            'interface': interface,
            'domain_id': ParameterValue(domain_id, value_type=int),
            'publish_odom_tf': True,
        }],
    )

    cmd_vel = Node(
        package='g1_ros2_bridge',
        executable='cmd_vel_bridge',
        name='g1_cmd_vel_bridge',
        output='screen',
        condition=IfCondition(enable_cmd_vel),
        parameters=[{
            'interface': interface,
            'domain_id': ParameterValue(domain_id, value_type=int),
            'require_enable': ParameterValue(require_enable, value_type=bool),
            'dry_run': ParameterValue(dry_run, value_type=bool),
        }],
    )

    loco = Node(
        package='g1_ros2_bridge',
        executable='loco_bridge',
        name='g1_loco_bridge',
        output='screen',
        condition=IfCondition(enable_loco),
        parameters=[{
            'interface': interface,
            'domain_id': ParameterValue(domain_id, value_type=int),
            'dry_run': ParameterValue(dry_run, value_type=bool),
        }],
    )

    realsense = Node(
        package='g1_ros2_bridge',
        executable='realsense_publisher',
        name='g1_realsense_publisher',
        output='screen',
        condition=IfCondition(enable_realsense),
    )

    pkg_share = get_package_share_directory('g1_ros2_bridge')
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'description.launch.py')),
        condition=IfCondition(enable_description),
        launch_arguments={
            'urdf_path': urdf_path,
        }.items(),
    )

    return LaunchDescription(args + [state, odom, cmd_vel, loco, realsense, description_launch])
