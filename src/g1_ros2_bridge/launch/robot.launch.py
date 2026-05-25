"""
robot.launch.py — Bring-up of the G1-side bridges.

Run this on the robot itself (or on any machine on the same DDS network).
Publishes /joint_states, /imu/data, /odom, /tf, /tf_static, /robot_description
and subscribes /cmd_vel + /g1/enable.

By default this launch ALSO runs `description.launch.py` (robot_state_publisher
plus the static TFs not in the URDF), so the robot is the single source of
truth for the TF tree. RViz on a laptop will then see a complete chain
(`odom -> base_link -> pelvis -> ...`) without needing any local URDF.

Args:
  interface           Network interface for Unitree DDS (default: $G1_INTERFACE)
  domain_id           ROS_DOMAIN_ID for Unitree DDS (default: 0)
  enable_odom         Run the SportModeState->/odom bridge (default: true)
  enable_cmd_vel      Run the /cmd_vel -> LocoClient bridge (default: true)
  enable_loco         Run the FSM / standing service bridge (default: true)
  enable_description  Run robot_state_publisher + static TFs (default: true)
  enable_camera       Launch the Intel realsense2_camera node (default: true).
                      Silently skipped if ros-foxy-realsense2-camera is not installed.
                      Publishes /camera/camera/{color,depth} images and point cloud.
  depth_profile       D435 depth stream resolution/framerate (default: 1280x720x30)
  pointcloud_enable   Publish /camera/camera/depth/color/points (default: true)
  require_enable      Require /g1/enable=true before cmd_vel passes through (default: true)
  dry_run             cmd_vel_bridge / loco_bridge log but do not send to the robot (default: false)
  quiet               Filter the rmw_cyclonedds discovery noise from each
                      bridge node's stderr (default: true)
  urdf_path           Path to the G1 URDF (forwarded to description.launch.py;
                      defaults to the bundled `g1_29dof.urdf`)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _build_nodes(context, *args, **kwargs):
    """Resolve LaunchConfigurations into concrete strings BEFORE constructing
    Node actions. We do this so the `prefix=` (which doesn't accept
    Substitutions in Foxy) can be set conditionally on `quiet`."""
    interface = LaunchConfiguration('interface')
    domain_id = LaunchConfiguration('domain_id')
    enable_odom = LaunchConfiguration('enable_odom')
    enable_cmd_vel = LaunchConfiguration('enable_cmd_vel')
    enable_loco = LaunchConfiguration('enable_loco')
    enable_description = LaunchConfiguration('enable_description')
    enable_camera = LaunchConfiguration('enable_camera')
    depth_profile = LaunchConfiguration('depth_profile')
    pointcloud_enable = LaunchConfiguration('pointcloud_enable')
    require_enable = LaunchConfiguration('require_enable')
    dry_run = LaunchConfiguration('dry_run')
    quiet = LaunchConfiguration('quiet')
    urdf_path = LaunchConfiguration('urdf_path')

    pkg_share = get_package_share_directory('g1_ros2_bridge')
    quiet_script = os.path.join(pkg_share, 'scripts', 'quiet_run.sh')
    quiet_str = context.perform_substitution(quiet)
    prefix = [quiet_script] if quiet_str.lower() in ('true', '1') else None

    state = Node(
        package='g1_ros2_bridge',
        executable='state_bridge',
        name='g1_state_bridge',
        output='screen',
        prefix=prefix,
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
        prefix=prefix,
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
        prefix=prefix,
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
        prefix=prefix,
        condition=IfCondition(enable_loco),
        parameters=[{
            'interface': interface,
            'domain_id': ParameterValue(domain_id, value_type=int),
            'dry_run': ParameterValue(dry_run, value_type=bool),
        }],
    )

    # realsense2_camera is an optional runtime dependency: if the package is not
    # installed the camera section is silently skipped and the other bridges
    # still come up. Install with: sudo apt install ros-foxy-realsense2-camera
    realsense_share = None
    try:
        realsense_share = get_package_share_directory('realsense2_camera')
    except Exception:
        realsense_share = None

    camera_actions = []
    if realsense_share is not None:
        camera_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(realsense_share, 'launch', 'rs_launch.py')),
            condition=IfCondition(enable_camera),
            launch_arguments={
                'depth_module.depth_profile': depth_profile,
                'pointcloud.enable': pointcloud_enable,
            }.items(),
        ))

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'description.launch.py')),
        condition=IfCondition(enable_description),
        launch_arguments={
            'urdf_path': urdf_path,
        }.items(),
    )

    return [state, odom, cmd_vel, loco, *camera_actions, description_launch]


def generate_launch_description():
    pkg_share = get_package_share_directory('g1_ros2_bridge')
    default_urdf = os.path.join(pkg_share, 'description', 'urdf', 'g1_29dof.urdf')

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
        DeclareLaunchArgument('enable_camera', default_value='true',
            description='Launch realsense2_camera (silently skipped if not installed)'),
        DeclareLaunchArgument('depth_profile', default_value='1280x720x30',
            description='RealSense depth stream profile (WxHxFPS)'),
        DeclareLaunchArgument('pointcloud_enable', default_value='true',
            description='Publish /camera/camera/depth/color/points'),
        DeclareLaunchArgument('require_enable', default_value='true'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('quiet', default_value='true',
            description='Filter rmw_cyclonedds discovery noise from bridge '
                        "node stderr (state_bridge, odom_bridge, etc.)"),
        DeclareLaunchArgument('urdf_path', default_value=default_urdf),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_build_nodes)])
