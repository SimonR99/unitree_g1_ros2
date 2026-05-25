"""
description.launch.py — robot_state_publisher (URDF + /tf_static) plus the
static TFs that aren't expressed in the G1 URDF.

The G1 29-DOF URDF and meshes ship inside this package
(`share/g1_ros2_bridge/description/`). robot_state_publisher publishes the
URDF as `/robot_description` and broadcasts every fixed joint on `/tf_static`
(which gives `pelvis -> torso_link -> ... -> mid360_link -> livox_frame`,
`pelvis -> torso_link -> d435_link`, `pelvis -> imu_in_pelvis`, ...).

This launch additionally emits the static TFs that the URDF does not declare
but which the bridges and sensors rely on:

  - base_link -> pelvis        (URDF root is `pelvis`; odom_bridge gives
                                `odom -> base_link`, so this closes the chain.)

The camera optical frames (`camera_color_optical_frame`,
`camera_depth_optical_frame`, …) are published by `realsense2_camera` itself
when the camera node is up — we don't duplicate them here.

Together with state_bridge (`pelvis -> imu_link`) and odom_bridge
(`odom -> base_link`), this gives a full TF chain to every URDF link
(`livox_frame`, every joint, the cameras, ...) as long as ONE machine on the
ROS network runs this launch.

Args:
  urdf_path     Path to the G1 URDF
                (default: `share/g1_ros2_bridge/description/urdf/g1_29dof.urdf`)
  quiet         Filter rmw_cyclonedds discovery noise from each child's
                stderr (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context, *args, **kwargs):
    pkg_share = get_package_share_directory('g1_ros2_bridge')
    quiet_script = os.path.join(pkg_share, 'scripts', 'quiet_run.sh')

    urdf_path = context.perform_substitution(LaunchConfiguration('urdf_path'))
    quiet_str = context.perform_substitution(LaunchConfiguration('quiet'))
    prefix = [quiet_script] if quiet_str.lower() in ('true', '1') else None

    if not os.path.isfile(urdf_path):
        raise RuntimeError(
            f"URDF not found at {urdf_path}. Pass urdf_path:=<file> on the "
            f"command line, or rebuild the package so the bundled URDF lands "
            f"in share/g1_ros2_bridge/description/urdf/.")
    with open(urdf_path, 'r') as f:
        urdf = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        prefix=prefix,
        parameters=[{'robot_description': urdf}],
    )

    # URDF root link is `pelvis`; odom_bridge publishes `odom -> base_link`.
    # Glue the two with an identity static TF.
    base_to_pelvis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_pelvis_static_tf',
        output='screen',
        prefix=prefix,
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'pelvis'],
    )

    return [rsp, base_to_pelvis]


def generate_launch_description():
    pkg_share = get_package_share_directory('g1_ros2_bridge')
    default_urdf = os.path.join(pkg_share, 'description', 'urdf', 'g1_29dof.urdf')

    args = [
        DeclareLaunchArgument('urdf_path', default_value=default_urdf),
        DeclareLaunchArgument('quiet', default_value='true'),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_build_nodes)])
