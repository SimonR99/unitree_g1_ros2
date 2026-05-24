#!/usr/bin/env python3
"""
cmd_vel_bridge: geometry_msgs/Twist on /cmd_vel → Unitree G1 LocoClient.Move()

Safety:
 - Watchdog: if no Twist message arrives within `watchdog_timeout_s`,
   send a zero velocity to the robot.
 - Optional enable latch: when `require_enable:=true`, the bridge ignores
   Twist messages until a std_msgs/Bool true is published on /g1/enable.
   Publishing false (or letting the latch time out via
   `enable_timeout_s`) disarms the bridge and sends a stop.

This bridge does NOT run the balance / FSM procedure. The robot must already
be in a state that accepts SetVelocity (typically FSM 500 "Start" / balance).
Use the companion `loco_bridge` (services on /g1/loco/...) or the Unitree app
to bring the robot up to that state first.
"""

# IMPORTANT: unitree_sdk2py imported BEFORE rclpy — see state_bridge.py comment.
# DDS init happens in main() via dds_init.init_dds_from_args().
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from g1_ros2_bridge.dds_init import prepare_dds, finalize_dds


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('g1_cmd_vel_bridge')

        # Note: 'interface'/'domain_id' are consumed in main() before rclpy.init.
        self.declare_parameter('interface', '')
        self.declare_parameter('domain_id', 0)
        self.declare_parameter('max_vx', 0.6)
        self.declare_parameter('max_vy', 0.4)
        self.declare_parameter('max_wz', 0.6)
        self.declare_parameter('watchdog_timeout_s', 0.5)
        self.declare_parameter('require_enable', True)
        self.declare_parameter('enable_timeout_s', 0.0)  # 0 = no timeout
        self.declare_parameter('dry_run', False)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.watchdog_timeout_s = float(self.get_parameter('watchdog_timeout_s').value)
        self.require_enable = bool(self.get_parameter('require_enable').value)
        self.enable_timeout_s = float(self.get_parameter('enable_timeout_s').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        # LocoClient creation is deferred to main() (after finalize_dds()) so
        # rmw_cyclonedds can claim the cyclonedds domain first. See dds_init.py.
        self.loco = None

        self.enabled = not self.require_enable
        self.last_twist_t = self.get_clock().now()
        self.last_enable_t = self.get_clock().now()
        self.stopped = True

        qos = QoSProfile(depth=10)
        self.create_subscription(Twist, '/cmd_vel', self._on_twist, qos)
        if self.require_enable:
            self.create_subscription(Bool, '/g1/enable', self._on_enable, qos)
        self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info(
            f"cmd_vel_bridge ready (require_enable={self.require_enable}, "
            f"max=[vx={self.max_vx}, vy={self.max_vy}, wz={self.max_wz}], "
            f"watchdog={self.watchdog_timeout_s}s)")

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def _on_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.get_logger().info("Enable latch ON — accepting /cmd_vel.")
        elif not msg.data and self.enabled:
            self.get_logger().info("Enable latch OFF — ignoring /cmd_vel, stopping.")
            self._stop()
        self.enabled = bool(msg.data)
        self.last_enable_t = self.get_clock().now()

    def _on_twist(self, msg: Twist):
        self.last_twist_t = self.get_clock().now()
        if not self.enabled:
            # Rate-limit this warning so it doesn't spam at 100 Hz publish rates.
            now = self.get_clock().now()
            since = (now - getattr(self, '_last_disabled_warn',
                                   rclpy.time.Time(seconds=0))).nanoseconds * 1e-9
            if since > 2.0:
                self.get_logger().warn(
                    "Got /cmd_vel but bridge is DISABLED. "
                    "Publish std_msgs/Bool true on /g1/enable to arm, "
                    "or relaunch with require_enable:=false.")
                self._last_disabled_warn = now
            return

        vx = self._clamp(msg.linear.x, -self.max_vx, self.max_vx)
        vy = self._clamp(msg.linear.y, -self.max_vy, self.max_vy)
        wz = self._clamp(msg.angular.z, -self.max_wz, self.max_wz)

        if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
            self._stop()
            return

        was_stopped = self.stopped
        self.stopped = False
        if self.dry_run:
            self.get_logger().info(f"[dry_run] Move(vx={vx:.3f}, vy={vy:.3f}, vyaw={wz:.3f})")
        else:
            if was_stopped:
                self.get_logger().info(
                    f"Forwarding /cmd_vel → robot (first sample: "
                    f"vx={vx:.3f}, vy={vy:.3f}, vyaw={wz:.3f})")
            self.loco.Move(vx=vx, vy=vy, vyaw=wz, continous_move=True)

    def _stop(self):
        if self.stopped:
            return
        self.stopped = True
        if self.dry_run:
            self.get_logger().info("[dry_run] StopMove()")
        else:
            self.get_logger().info("Stopping robot (zero Twist or watchdog timeout).")
            self.loco.StopMove()

    def _watchdog_tick(self):
        now = self.get_clock().now()
        if (now - self.last_twist_t).nanoseconds * 1e-9 > self.watchdog_timeout_s:
            self._stop()
        if (self.require_enable and self.enable_timeout_s > 0.0 and self.enabled
                and (now - self.last_enable_t).nanoseconds * 1e-9 > self.enable_timeout_s):
            self.get_logger().warn(
                f"No /g1/enable heartbeat in {self.enable_timeout_s}s — disarming.")
            self.enabled = False
            self._stop()


def main(args=None):
    domain, iface = prepare_dds()
    rclpy.init(args=args)
    node = CmdVelBridge()
    finalize_dds(domain, iface)
    if node.dry_run:
        node.get_logger().warn("dry_run:=true — Twist messages will be logged but NOT sent to the robot.")
    else:
        node.loco = LocoClient()
        node.loco.SetTimeout(0.5)
        node.loco.Init()
    node.get_logger().info(
        f"DDS up (domain={domain}, interface='{iface or '<default>'}'); cmd_vel_bridge ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
