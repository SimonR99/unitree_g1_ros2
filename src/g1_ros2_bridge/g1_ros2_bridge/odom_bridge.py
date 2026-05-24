#!/usr/bin/env python3
"""
odom_bridge: Unitree G1 `rt/sportmodestate` (unitree_go SportModeState_) →
             nav_msgs/Odometry on /odom
             + TF odom → base_frame

NOTE: SportModeState gives position[3], velocity[3], yaw_speed, body_height, and
the IMU quaternion via imu_state. We use that as the odom orientation and the
provided position/velocity directly. There is no covariance from the robot, so
we leave covariance fields at 0 (consumers like robot_localization should treat
the values as approximate).
"""

# IMPORTANT: unitree_sdk2py imported BEFORE rclpy — see state_bridge.py comment.
# DDS init happens in main() via dds_init.init_dds_from_args().
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from g1_ros2_bridge.dds_init import prepare_dds, finalize_dds


class OdomBridge(Node):
    def __init__(self):
        super().__init__('g1_odom_bridge')

        # Note: 'interface'/'domain_id' are consumed in main() before rclpy.init.
        self.declare_parameter('interface', '')
        self.declare_parameter('domain_id', 0)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_odom_tf', True)

        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').get_parameter_value().bool_value

        qos = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.tf_br = TransformBroadcaster(self)

        # Subscriber attached from main() after rclpy/RMW domain is up.
        self.sub = None

    def _on_sportmode(self, msg: SportModeState_):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(msg.position[0])
        odom.pose.pose.position.y = float(msg.position[1])
        odom.pose.pose.position.z = float(msg.position[2])

        q = msg.imu_state.quaternion  # [w, x, y, z]
        odom.pose.pose.orientation.w = float(q[0])
        odom.pose.pose.orientation.x = float(q[1])
        odom.pose.pose.orientation.y = float(q[2])
        odom.pose.pose.orientation.z = float(q[3])

        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = float(msg.velocity[2])
        odom.twist.twist.angular.z = float(msg.yaw_speed)

        self.odom_pub.publish(odom)

        if self.publish_odom_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = odom.pose.pose.position.x
            t.transform.translation.y = odom.pose.pose.position.y
            t.transform.translation.z = odom.pose.pose.position.z
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_br.sendTransform(t)


def main(args=None):
    domain, iface = prepare_dds()
    rclpy.init(args=args)
    node = OdomBridge()
    finalize_dds(domain, iface)
    node.sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    node.sub.Init(node._on_sportmode)
    node.get_logger().info(
        f"DDS up (domain={domain}, interface='{iface or '<default>'}'); "
        "subscribed to rt/sportmodestate → /odom")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
