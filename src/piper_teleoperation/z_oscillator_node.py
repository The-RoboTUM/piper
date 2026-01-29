#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class PiperZOscillator(Node):
    def __init__(self):
        super().__init__('piper_z_oscillator')
        
        #Human hand extend initially
        self.robot_hand_start_pose = np.array([0.5, 0.0, 0.15]) #Robot hand starting pose in Cartesian coordinates
        self.human_hand_extended = False #Check whether we as human extend out our hand towards the robot
        self.target_pub = self.create_publisher(Point, '/piper_cartesian_target', 10)

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
        
        # Hand Detection
        self.hand_present = False # Added for Hand Detection
        self.last_true_time = 0.0 # Added for Hand Detection
        self.hold_time = 1.0 # Added for Hand Detection
        self.hand_sub = self.create_subscription(Bool, '/hand/semantic_detected', self.hand_callback, 10) # Added for Hand Detection
        
        # Send attract pose once at startup
        msg = Point()
        msg.x, msg.y, msg.z = self.robot_hand_start_pose.tolist()
        self.target_pub.publish(msg)

        self.get_logger().info(f"🧲 Moving to attract pose: {self.robot_hand_start_pose}")
    
    def hand_callback(self, msg: Bool): # Entire method is added for Hand Detection
        now = self.get_clock().now().nanoseconds * 1e-9 

        if msg.data: 
            self.hand_present = True 
            self.last_true_time = now 
        else:
            # only release if false persists long enough 
            if now - self.last_true_time > self.hold_time: 
                self.hand_present = False 

    """
    def loop(self):
        if not self.hand_present: # Added for Hand Detection
            return # Added for Hand Detection
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
    """

    def loop(self):
        # --------------------------
        # STATE: No hand detected
        # --------------------------
        if not self.hand_present:
            if not self.human_hand_extended:
                # Go back to attract pose
                msg = Point()
                msg.x, msg.y, msg.z = self.robot_hand_start_pose.tolist()
                self.target_pub.publish(msg)

                self.get_logger().info("↩️ Returning to attract pose")
                self.human_hand_extended = True

            return  # no shaking

        # --------------------------
        # STATE: Hand detected
        # --------------------------
        self.human_hand_extended = False

        # Publish Z oscillation
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

