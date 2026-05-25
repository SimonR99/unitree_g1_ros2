"""
description.launch.py — robot_state_publisher (URDF + /tf_static) plus the
static TFs that aren't expressed in the G1 URDF.

The G1 29-DOF URDF and meshes ship inside this package
(`share/g1_ros2_bridge/description/`). robot_state_publisher publishes the
URDF as `/robot_description` and broadcasts every fixed joint on `/tf_static`.

This launch additionally emits the static TFs that the URDF does not declare
but which the bridges and sensors rely on:

  - world  -> odom       (identity; anchors the TF tree so that any RViz
                          fixed-frame choice — world, odom, or pelvis —
                          resolves without error)
  - base_link -> pelvis  (URDF root is `pelvis`; odom_bridge gives
                          `odom -> base_link`, so this closes the chain.)

The full static chain is therefore:
  world -> odom -> base_link -> pelvis -> [all URDF links]

odom_bridge updates `odom -> base_link` dynamically from SportModeState.

The camera optical frames (`camera_color_optical_frame`,
`camera_depth_optical_frame`, …) are published by `realsense2_camera` itself
when the camera node is up — we don't duplicate them here.

Args:
  urdf_path     Path to the G1 URDF
                (default: `share/g1_ros2_bridge/description/urdf/g1_29dof.urdf`)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_nodes(context, *args, **kwargs):
    urdf_path = context.perform_substitution(LaunchConfiguration('urdf_path'))

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
        parameters=[{'robot_description': urdf}],
    )

    # world -> odom: identity static TF so every fixed-frame choice in RViz
    # (world, odom, pelvis, map …) reaches the robot's TF tree without errors.
    # odom_bridge overwrites odom -> base_link dynamically once the robot is
    # connected; world -> odom being static is fine for localisation-free use.
    world_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom'],
    )

    # URDF root link is `pelvis`; odom_bridge publishes `odom -> base_link`.
    # Glue the two with an identity static TF.
    base_to_pelvis = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_pelvis_static_tf',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'pelvis'],
    )

    return [rsp, world_to_odom, base_to_pelvis]


def generate_launch_description():
    pkg_share = get_package_share_directory('g1_ros2_bridge')
    default_urdf = os.path.join(pkg_share, 'description', 'urdf', 'g1_29dof.urdf')

    args = [
        DeclareLaunchArgument('urdf_path', default_value=default_urdf),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_build_nodes)])
