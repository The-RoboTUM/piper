#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class PiperZOscillator(Node):
    def __init__(self):
        super().__init__('piper_z_oscillator')

        # Publisher to Cartesian delta topic
        self.pub = self.create_publisher(
            Point,
            '/piper_cartesian_delta',
            10
        )

        # Parameters (Gazebo-safe)
        self.step = 0.005     # 1 cm per command
        self.max_steps = 30  # 5 cm total
        self.counter = 0
        self.direction = 1  # +1 or -1

        # Timer: 10 Hz
        self.timer = self.create_timer(0.015, self.loop)

        self.get_logger().info("🚀 Piper Z Oscillator started")
        self.get_logger().info("⬆️⬇️  Oscillating ±5 cm in Z")

    def loop(self):
        msg = Point()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = self.direction * self.step

        self.pub.publish(msg)

        self.counter += 1

        if self.counter >= self.max_steps:
            self.direction *= -1
            self.counter = 0
            self.get_logger().info(
                f"🔁 Direction switched → {'UP' if self.direction > 0 else 'DOWN'}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = PiperZOscillator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

