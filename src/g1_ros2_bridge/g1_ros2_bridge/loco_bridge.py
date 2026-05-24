#!/usr/bin/env python3
"""
loco_bridge: ROS 2 wrapper around `unitree_sdk2py.g1.loco.LocoClient`.

Exposes the G1 Finite State Machine and standing-mode controls as ROS 2
services and topics, so you can drive the robot to FSM 500 ("Start") — the
state required for `/cmd_vel` to actually move the robot — without going
through the Unitree app.

Services (`std_srvs/srv/Trigger`):
  /g1/loco/start             → SetFsmId(500)   Start / balance / locomotion
  /g1/loco/damp              → SetFsmId(1)     Damping (joints soft, robot collapses)
  /g1/loco/sit               → SetFsmId(3)     Sit
  /g1/loco/zero_torque       → SetFsmId(0)     Zero-torque (limp)
  /g1/loco/squat_to_stand    → SetFsmId(706)
  /g1/loco/lie_to_stand      → SetFsmId(702)
  /g1/loco/stand_to_squat    → SetFsmId(706)
  /g1/loco/high_stand        → SetStandHeight(MAX)
  /g1/loco/low_stand         → SetStandHeight(0)
  /g1/loco/stop_move         → SetVelocity(0,0,0)

Topics (subscribers):
  /g1/loco/set_fsm_id          std_msgs/Int32     arbitrary FSM ID
  /g1/loco/set_balance_mode    std_msgs/Int32
  /g1/loco/set_stand_height    std_msgs/Float32   meters

Topics (publishers, optional):
  /g1/loco/last_command        std_msgs/String    last successful command
                               (handy for logging / dashboards)

The node uses the same DDS-init shim as the other bridges, so it can run
side-by-side with `state_bridge`, `odom_bridge`, and `cmd_vel_bridge` in the
same process tree. `dry_run:=true` logs every call but doesn't talk to the
robot.
"""

# IMPORTANT: unitree_sdk2py imported BEFORE rclpy — see state_bridge.py comment.
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Int32, Float32, String
from std_srvs.srv import Trigger

from g1_ros2_bridge.dds_init import prepare_dds, finalize_dds


class LocoBridge(Node):
    def __init__(self):
        super().__init__('g1_loco_bridge')

        # Note: 'interface'/'domain_id' are consumed in main() before rclpy.init.
        self.declare_parameter('interface', '')
        self.declare_parameter('domain_id', 0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('rpc_timeout_s', 1.0)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.rpc_timeout_s = float(self.get_parameter('rpc_timeout_s').value)

        # LocoClient is created in main() after finalize_dds(); see dds_init.py.
        self.loco = None

        # Each entry: (service_name, human_label, callable_factory).
        # `callable_factory` returns a 0-arg lambda that hits the SDK; we wrap
        # late so unit-testing without a robot still makes sense.
        self._service_specs = [
            ('~/start',           'Start (FSM 500)',          lambda c: c.Start),
            ('~/damp',            'Damp (FSM 1)',             lambda c: c.Damp),
            ('~/sit',             'Sit (FSM 3)',              lambda c: c.Sit),
            ('~/zero_torque',     'ZeroTorque (FSM 0)',       lambda c: c.ZeroTorque),
            ('~/squat_to_stand',  'Squat2StandUp (FSM 706)',  lambda c: c.Squat2StandUp),
            ('~/lie_to_stand',    'Lie2StandUp (FSM 702)',    lambda c: c.Lie2StandUp),
            ('~/stand_to_squat',  'StandUp2Squat (FSM 706)',  lambda c: c.StandUp2Squat),
            ('~/high_stand',      'HighStand',                lambda c: c.HighStand),
            ('~/low_stand',       'LowStand',                 lambda c: c.LowStand),
            ('~/stop_move',       'StopMove',                 lambda c: c.StopMove),
        ]
        self._services = []
        for srv_name, label, fac in self._service_specs:
            self._services.append(self.create_service(
                Trigger, srv_name, self._make_trigger_cb(label, fac)))

        qos = QoSProfile(depth=10)
        self.create_subscription(Int32, '~/set_fsm_id',
                                 self._on_set_fsm_id, qos)
        self.create_subscription(Int32, '~/set_balance_mode',
                                 self._on_set_balance_mode, qos)
        self.create_subscription(Float32, '~/set_stand_height',
                                 self._on_set_stand_height, qos)
        self.last_cmd_pub = self.create_publisher(String, '~/last_command', qos)

        self.get_logger().info(
            f"loco_bridge ready (dry_run={self.dry_run}). "
            "Services: " + ", ".join(s for s, _, _ in self._service_specs))

    def _publish_last(self, label: str) -> None:
        msg = String()
        msg.data = label
        self.last_cmd_pub.publish(msg)

    def _make_trigger_cb(self, label, fac):
        def _cb(_request, response):
            try:
                code = self._invoke(label, fac)
            except Exception as e:
                response.success = False
                response.message = f"{label} raised {type(e).__name__}: {e}"
                self.get_logger().error(response.message)
                return response
            response.success = (code == 0)
            response.message = f"{label} → code={code}"
            return response
        return _cb

    def _invoke(self, label: str, fac, *call_args):
        """Call a LocoClient method (resolved via `fac(self.loco)`), honouring
        dry-run and logging the result. Returns the SDK return code, or 0
        in dry-run."""
        if self.dry_run or self.loco is None:
            self.get_logger().info(f"[dry_run] {label}({', '.join(map(str, call_args))})")
            self._publish_last(f"[dry_run] {label}")
            return 0
        method = fac(self.loco)
        code = method(*call_args)
        # Some helpers on LocoClient (Damp, Start, ...) call SetFsmId() but
        # don't return its code; treat None as 0.
        if code is None:
            code = 0
        if code == 0:
            self.get_logger().info(f"{label} OK")
        else:
            self.get_logger().warn(f"{label} returned non-zero code={code}")
        self._publish_last(label)
        return code

    def _on_set_fsm_id(self, msg: Int32):
        self._invoke(f"SetFsmId({msg.data})",
                     lambda c: c.SetFsmId, int(msg.data))

    def _on_set_balance_mode(self, msg: Int32):
        self._invoke(f"SetBalanceMode({msg.data})",
                     lambda c: c.SetBalanceMode, int(msg.data))

    def _on_set_stand_height(self, msg: Float32):
        self._invoke(f"SetStandHeight({msg.data:.3f})",
                     lambda c: c.SetStandHeight, float(msg.data))


def main(args=None):
    domain, iface = prepare_dds()
    rclpy.init(args=args)
    node = LocoBridge()
    finalize_dds(domain, iface)
    if node.dry_run:
        node.get_logger().warn(
            "dry_run:=true — service/topic calls will be logged but NOT sent to the robot.")
    else:
        node.loco = LocoClient()
        node.loco.SetTimeout(node.rpc_timeout_s)
        node.loco.Init()
    node.get_logger().info(
        f"DDS up (domain={domain}, interface='{iface or '<default>'}'); loco_bridge ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
