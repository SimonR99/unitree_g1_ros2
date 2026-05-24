#!/usr/bin/env python3
"""
state_bridge: Unitree G1 `rt/lowstate` (unitree_hg LowState_) →
              sensor_msgs/JointState on /joint_states
              sensor_msgs/Imu on /imu/data
              + TF pelvis → imu_link
"""

# IMPORTANT: unitree_sdk2py must be imported BEFORE rclpy / sensor_msgs, and
# ChannelFactoryInitialize must run BEFORE any rclpy.Node is constructed.
# Otherwise FastRTPS (Foxy default RMW) claims the local DDS domain first and
# cyclonedds fails with "create domain error" / DDSException PRECONDITION_NOT_MET.
# DDS is initialised in main() via dds_init.init_dds_from_args().
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from g1_ros2_bridge.dds_init import prepare_dds, finalize_dds
from g1_ros2_bridge.joint_names import JOINT_INDICES, JOINT_NAMES, JOINT_INDEX_TO_NAME


class StateBridge(Node):
    def __init__(self):
        super().__init__('g1_state_bridge')

        # Note: 'interface' and 'domain_id' are consumed BEFORE rclpy.init by
        # dds_init.init_dds_from_args(). They are declared here too so they
        # show up in `ros2 param list` and YAML overrides.
        self.declare_parameter('interface', '')
        self.declare_parameter('domain_id', 0)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('base_frame_id', 'pelvis')
        self.declare_parameter('publish_imu_tf', True)

        self.imu_frame = self.get_parameter('imu_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value
        self.publish_imu_tf = self.get_parameter('publish_imu_tf').get_parameter_value().bool_value

        qos = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', qos)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', qos)
        self.tf_br = TransformBroadcaster(self)

        self._js = JointState()
        self._js.name = JOINT_NAMES

        # Subscriber set up by `attach_dds()`, called from main() after the
        # rclpy/RMW domain is in place.
        self.sub = None

    def _on_lowstate(self, msg: LowState_):
        now = self.get_clock().now().to_msg()

        # IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = self.imu_frame
        q = msg.imu_state.quaternion  # [w, x, y, z]
        imu.orientation.w = float(q[0])
        imu.orientation.x = float(q[1])
        imu.orientation.y = float(q[2])
        imu.orientation.z = float(q[3])
        imu.angular_velocity.x = float(msg.imu_state.gyroscope[0])
        imu.angular_velocity.y = float(msg.imu_state.gyroscope[1])
        imu.angular_velocity.z = float(msg.imu_state.gyroscope[2])
        imu.linear_acceleration.x = float(msg.imu_state.accelerometer[0])
        imu.linear_acceleration.y = float(msg.imu_state.accelerometer[1])
        imu.linear_acceleration.z = float(msg.imu_state.accelerometer[2])
        self.imu_pub.publish(imu)

        if self.publish_imu_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.base_frame
            t.child_frame_id = self.imu_frame
            t.transform.rotation = imu.orientation
            self.tf_br.sendTransform(t)

        # Joint states
        positions, velocities, efforts = [], [], []
        for idx in JOINT_INDICES:
            if idx < len(msg.motor_state):
                m = msg.motor_state[idx]
                positions.append(float(m.q))
                velocities.append(float(m.dq))
                efforts.append(float(m.tau_est))
            else:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)
        self._js.header.stamp = now
        self._js.position = positions
        self._js.velocity = velocities
        self._js.effort = efforts
        self.joint_pub.publish(self._js)


def main(args=None):
    # Step 1: parse args, set CYCLONEDDS_URI, monkey-patch the SDK. Must happen
    # before rclpy/cyclonedds imports do anything.
    domain, iface = prepare_dds()

    rclpy.init(args=args)
    node = StateBridge()
    # Step 2: now rmw_cyclonedds has created the domain — join it from the SDK
    # side and wire up the rt/lowstate subscriber.
    finalize_dds(domain, iface)
    node.sub = ChannelSubscriber("rt/lowstate", LowState_)
    node.sub.Init(node._on_lowstate)
    node.get_logger().info(
        f"DDS up (domain={domain}, interface='{iface or '<default>'}'); "
        "subscribed to rt/lowstate → /joint_states + /imu/data")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
