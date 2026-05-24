#!/usr/bin/env python3
"""
realsense_publisher: pyrealsense2 → sensor_msgs/Image + CameraInfo

Publishes:
  /camera/color/image_raw          sensor_msgs/Image (bgr8)
  /camera/color/camera_info        sensor_msgs/CameraInfo
  /camera/depth/image_raw          sensor_msgs/Image (16UC1, mm)
  /camera/depth/camera_info        sensor_msgs/CameraInfo

Falls back gracefully if pyrealsense2 is not installed or no camera is
attached — the node logs and exits cleanly rather than crashing the launch.

Foxy ships no realsense2_camera package by default, so this node is the
recommended path for getting standard-topic images out of the robot's D435i.
If you have a properly installed realsense2_camera, prefer that instead.
"""

import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Image, CameraInfo


def _make_image(stamp, frame_id, encoding, step, data, h, w):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = h
    msg.width = w
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = step
    msg.data = data
    return msg


def _make_camera_info(stamp, frame_id, intr):
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = intr.height
    msg.width = intr.width
    msg.distortion_model = 'plumb_bob'
    # pyrealsense2 distortion coeffs are 5 floats for Brown-Conrady/Plumb-Bob.
    msg.d = [float(c) for c in intr.coeffs[:5]]
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy
    msg.k = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    return msg


class RealSensePublisher(Node):
    def __init__(self):
        super().__init__('g1_realsense_publisher')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('color_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('depth_frame_id', 'camera_depth_optical_frame')
        self.declare_parameter('enable_color', True)
        self.declare_parameter('enable_depth', True)

        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        fps = int(self.get_parameter('fps').value)
        self.color_frame = self.get_parameter('color_frame_id').get_parameter_value().string_value
        self.depth_frame = self.get_parameter('depth_frame_id').get_parameter_value().string_value
        self.enable_color = bool(self.get_parameter('enable_color').value)
        self.enable_depth = bool(self.get_parameter('enable_depth').value)

        try:
            import pyrealsense2 as rs  # noqa: F401
        except ImportError:
            self.get_logger().error(
                "pyrealsense2 is not installed. Install with:\n"
                "    python3 -m pip install --user pyrealsense2\n"
                "or use the system realsense2_camera package instead.")
            raise

        self.rs = rs
        cfg = rs.config()
        if self.enable_color:
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        if self.enable_depth:
            cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)

        self.pipeline = rs.pipeline()
        try:
            profile = self.pipeline.start(cfg)
        except RuntimeError as e:
            self.get_logger().error(f"Failed to start RealSense pipeline: {e}")
            raise

        self.color_intr = None
        self.depth_intr = None
        if self.enable_color:
            self.color_intr = profile.get_stream(rs.stream.color) \
                .as_video_stream_profile().get_intrinsics()
        if self.enable_depth:
            self.depth_intr = profile.get_stream(rs.stream.depth) \
                .as_video_stream_profile().get_intrinsics()

        qos = QoSProfile(depth=5)
        if self.enable_color:
            self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', qos)
            self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos)
        if self.enable_depth:
            self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos)
            self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos)

        # Drive the loop slightly faster than the camera FPS to avoid stalling.
        self.create_timer(1.0 / max(fps, 1) * 0.5, self._tick)
        self.get_logger().info(
            f"RealSense streaming {w}x{h}@{fps} "
            f"(color={self.enable_color}, depth={self.enable_depth})")

    def _tick(self):
        try:
            frames = self.pipeline.poll_for_frames()
        except Exception as e:
            self.get_logger().warn(f"poll_for_frames failed: {e}")
            return
        if not frames:
            return

        stamp = self.get_clock().now().to_msg()

        if self.enable_color:
            cf = frames.get_color_frame()
            if cf:
                arr = np.asanyarray(cf.get_data())
                h, w = arr.shape[:2]
                step = arr.strides[0]
                msg = _make_image(stamp, self.color_frame, 'bgr8', step, arr.tobytes(), h, w)
                self.color_pub.publish(msg)
                self.color_info_pub.publish(_make_camera_info(stamp, self.color_frame, self.color_intr))

        if self.enable_depth:
            df = frames.get_depth_frame()
            if df:
                arr = np.asanyarray(df.get_data())
                h, w = arr.shape[:2]
                step = arr.strides[0]
                msg = _make_image(stamp, self.depth_frame, '16UC1', step, arr.tobytes(), h, w)
                self.depth_pub.publish(msg)
                self.depth_info_pub.publish(_make_camera_info(stamp, self.depth_frame, self.depth_intr))

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = RealSensePublisher()
    except Exception as e:
        # Exit cleanly so a launch file with this node optional won't bring down siblings.
        print(f"[realsense_publisher] startup failed: {e}", file=sys.stderr)
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
