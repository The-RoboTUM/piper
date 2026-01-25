#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys, termios, tty, select, math
from std_msgs.msg import String


def get_key(settings):
    """
    从终端读取一个按键（非阻塞）
    Read a single key press from terminal (non-blocking)
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.03)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class PiperTeleopJoint(Node):
    def __init__(self):
        """
        Piper 机械臂 关节级（Joint Space）键盘遥操作节点
        Keyboard teleoperation node for Piper robot (joint space control)
        """
        super().__init__('piper_teleop_joint')

        # 发布关节指令（JointState），用于直接控制各关节角度
        # Publisher for joint commands (JointState messages)
        self.pub = self.create_publisher(JointState, '/joint_commands_joi', 10)

        # 发布当前控制模式（joint / cartesian）
        # Publish current control mode (joint or cartesian)
        self.mode_pub = self.create_publisher(String, '/control_mode', 10)
        
        # 订阅真实机器人或 Gazebo 的当前关节状态
        # Subscribe to current joint states from robot / simulation
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )

        # 当前控制模式（默认 Cartesian）
        # Current control mode (default: cartesian)
        self.current_mode = "cartesian"
        
        # 定时器：每 30ms 执行一次 update_cmd
        # Timer running at ~33 Hz for keyboard update loop
        self.timer = self.create_timer(0.03, self.update_cmd)

        # 关节角初始化（占位），会在第一次接收到 joint_states 后更新
        # Joint positions placeholder, replaced after first joint_states message
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state_received = False
        
        # 机械臂关节角度限制（单位：弧度）
        # Joint limits in radians
        self.joint_limits = [
            (-2.618, 2.618),    # Joint 1
            (0.0, math.pi),     # Joint 2  
            (-math.pi, 0.0),    # Joint 3
            (-2.967, 2.967),    # Joint 4
            (-1.2, 1.2),        # Joint 5
            (-1.22, 1.22),      # Joint 6
            (0.0, 0.04)         # Gripper（夹爪）
        ]
        
        # 单次按键的角度增量（弧度）
        # Angle increment per key press (radians)
        self.step = 0.05

        # 夹爪施加的力（effort）
        # Gripper effort value
        self.gripper_effort = 1.0
        
        self.get_logger().info("等待接收机器人当前关节位置...")
        # Waiting for current joint positions from robot

    def switch_mode(self):
        """
        在 joint / cartesian 控制模式之间切换
        Toggle between joint and cartesian control modes
        """
        self.current_mode = "cartesian" if self.current_mode == "joint" else "joint"
        mode_msg = String(data=self.current_mode)
        self.mode_pub.publish(mode_msg)
        self.get_logger().info(f"当前模式: {self.current_mode.upper()}")
        # Log current control mode

    def joint_state_callback(self, msg):
        """
        接收机器人当前真实关节角度
        Callback to receive current joint positions
        """
        if not self.joint_state_received:
            self.joints = list(msg.position)
            self.joint_state_received = True

            self.get_logger().info(
                f"当前关节位置: {[f'{x:.3f}' for x in self.joints]}"
            )
            self.get_logger().info(
                "Piper Teleop 已启动。\n"
                "Q/A, W/S, E/D, R/F, T/G, Y/H, U/J → joints 1–7\n"
                "ESC 退出"
            )
            # Teleop instructions printed once after initialization

    def apply_joint_limits(self, joint_idx, value):
        """
        对关节角度进行限幅，防止超出机械极限
        Clamp joint value within predefined limits
        """
        min_val, max_val = self.joint_limits[joint_idx]
        return max(min_val, min(value, max_val))

    def update_cmd(self):
        """
        主循环：读取键盘 → 更新关节角度 → 发布 JointState
        Main loop: read keyboard, update joints, publish commands
        """
        # 尚未接收到关节状态前，不允许控制
        # Do nothing until first joint_states is received
        if not self.joint_state_received:
            return

        settings = termios.tcgetattr(sys.stdin)
        key = get_key(settings)

        # ESC 键退出程序
        # Exit on ESC key
        if key == '\x1b':
            self.get_logger().info("退出遥操作程序")
            raise KeyboardInterrupt

        # ========== 各关节键位映射 ==========
        # Key bindings for each joint

        # joint 1
        if key == 'q': 
            self.joints[0] = self.apply_joint_limits(0, self.joints[0] + self.step)
        if key == 'a': 
            self.joints[0] = self.apply_joint_limits(0, self.joints[0] - self.step)

        # joint 2
        if key == 'w': 
            self.joints[1] = self.apply_joint_limits(1, self.joints[1] + self.step)
        if key == 's': 
            self.joints[1] = self.apply_joint_limits(1, self.joints[1] - self.step)

        # joint 3
        if key == 'e': 
            self.joints[2] = self.apply_joint_limits(2, self.joints[2] + self.step)
        if key == 'd': 
            self.joints[2] = self.apply_joint_limits(2, self.joints[2] - self.step)

        # joint 4
        if key == 'r': 
            self.joints[3] = self.apply_joint_limits(3, self.joints[3] + self.step)
        if key == 'f': 
            self.joints[3] = self.apply_joint_limits(3, self.joints[3] - self.step)

        # joint 5
        if key == 't': 
            self.joints[4] = self.apply_joint_limits(4, self.joints[4] + self.step)
        if key == 'g': 
            self.joints[4] = self.apply_joint_limits(4, self.joints[4] - self.step)

        # joint 6
        if key == 'y': 
            self.joints[5] = self.apply_joint_limits(5, self.joints[5] + self.step)
        if key == 'h': 
            self.joints[5] = self.apply_joint_limits(5, self.joints[5] - self.step)

        # gripper（夹爪）
        # Gripper open / close
        if key == 'u': 
            self.joints[6] = min(self.joints[6] + 0.005, 0.04)
        if key == 'j': 
            self.joints[6] = max(self.joints[6] - 0.005, 0.0)

        # 模式切换
        # Switch control mode
        if key == 'm': 
            self.switch_mode()

        # ========= 发布关节指令 =========
        # Always publish to hold current position
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        msg.position = self.joints
        msg.velocity = [0.0]*6 + [10.0]
        msg.effort = [0.0]*6 + [self.gripper_effort]
        self.pub.publish(msg)


def main(args=None):
    """
    ROS2 节点入口
    ROS2 node entry point
    """
    rclpy.init(args=args)
    node = PiperTeleopJoint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

