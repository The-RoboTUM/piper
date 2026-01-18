#!/usr/bin/env python3
from __future__ import annotations

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class DepthGateNode(Node):
    """
    Minimal depth-based hand presence detector for Intel RealSense D405.

    Subscribes:  /camera/depth/image_rect_raw (or whatever you set)
    Publishes:   /depth/gate  (std_msgs/Bool)

    Detection rule (default):
      - Convert depth image to meters (handles 16UC1 in mm and 32FC1 in meters)
      - Count pixels with depth in [min_depth_m, max_depth_m]
      - If count >= min_pixels -> object detected True
    """

    def __init__(self) -> None:
        super().__init__("depth_gate_node")

        # ---- Parameters (tune these later) ----
        self.declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("min_depth_m", 0.10)     # D405 works best close-up
        self.declare_parameter("max_depth_m", 0.70)
        self.declare_parameter("min_pixels", 2500)      # threshold for "something close"
        self.declare_parameter("publish_rate_hz", 30.0) # publish even if no new frames
        self.declare_parameter("debug_log", False)

        self.depth_topic: str = self.get_parameter("depth_topic").value
        self.min_depth_m: float = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m: float = float(self.get_parameter("max_depth_m").value)
        self.min_pixels: int = int(self.get_parameter("min_pixels").value)
        self.publish_rate_hz: float = float(self.get_parameter("publish_rate_hz").value)
        self.debug_log: bool = bool(self.get_parameter("debug_log").value)

        # ---- ROS interfaces ----
        self.pub = self.create_publisher(Bool, "/presence/far", 10)
        self.sub = self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)

        self.last_detected: bool = False
        self.last_close_pixels: int = 0

        # Publish periodically so downstream nodes get a steady signal
        period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Depth Gate up. Subscribing to: {self.depth_topic} | Publishing: /presence/far\n"
            f"Range: [{self.min_depth_m:.2f}, {self.max_depth_m:.2f}] m | min_pixels: {self.min_pixels}"
        )

    def on_depth(self, msg: Image) -> None:
        # Supported encodings typically: "16UC1" (mm) or "32FC1" (meters)
        h, w = msg.height, msg.width

        if msg.encoding == "16UC1":
            # depth in millimeters -> meters
            depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w).astype(np.float32) / 1000.0
        elif msg.encoding == "32FC1":
            # depth in meters
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}. Expected 16UC1 or 32FC1.")
            return

        # Filter out invalid values (0, NaN, inf)
        valid = np.isfinite(depth) & (depth > 0.0)

        close = valid & (depth >= self.min_depth_m) & (depth <= self.max_depth_m)
        close_pixels = int(np.count_nonzero(close))

        detected = close_pixels >= self.min_pixels

        self.last_detected = detected
        self.last_close_pixels = close_pixels

        if self.debug_log:
            self.get_logger().info(f"Got depth frame: {msg.width}x{msg.height}, encoding={msg.encoding}")
            self.get_logger().info(f"close_pixels={close_pixels} detected={detected}")



    def on_timer(self) -> None:
        out = Bool()
        out.data = bool(self.last_detected)
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = DepthGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
