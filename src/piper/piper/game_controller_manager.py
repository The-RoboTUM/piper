#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
import pygame
import os


class PiperTeleopGamepad(Node):
    def __init__(self):
        # 初始化 ROS2 节点，节点名为 piper_teleop_gamepad
        # Initialize ROS2 node with name "piper_teleop_gamepad"
        super().__init__('piper_teleop_gamepad')

        # ============================================================
        # 发布器（Publishers）
        # ============================================================

        # 发布笛卡尔空间的位移增量 Δx, Δy, Δz
        # Publish Cartesian delta commands (Δx, Δy, Δz)
        self.pub_cart = self.create_publisher(
            Point, '/piper_cartesian_delta', 10)

        # 发布关节空间的增量 Δq（JointState 格式）
        # Publish joint-space incremental commands (Δq)
        self.pub_joint = self.create_publisher(
            JointState, '/joint_commands_joi', 10)

        # 发布夹爪控制（绝对值）
        # Publish gripper command (absolute position/effort)
        self.pub_grip = self.create_publisher(
            Float64, '/gripper_command', 10)

        # 发布当前控制模式（cartesian / joint）
        # Publish current control mode ("cartesian" or "joint")
        self.pub_mode = self.create_publisher(
            String, '/control_mode', 10)

        # ============================================================
        # 订阅器（Subscriber）
        # ============================================================

        # 接收外部节点发来的控制模式切换指令
        # Receive external control mode override
        self.sub_mode = self.create_subscription(
            String, '/control_mode', self.mode_callback, 10)

        # ============================================================
        # 初始化 pygame 与手柄
        # ============================================================

        # 初始化 pygame 系统
        # Initialize pygame
        pygame.init()
        pygame.joystick.init()

        # 检查是否检测到手柄
        # Check if any gamepad is detected
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("❌ 未检测到手柄 | No gamepad detected")
            exit()

        # 使用第一个手柄
        # Use the first detected joystick
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(
            f"✅ 已检测到手柄 | Gamepad detected: {self.joy.get_name()}")

        # ============================================================
        # 控制参数（Parameters）
        # ============================================================

        # 单次关节增量（弧度）
        # Joint increment per step (radians)
        self.joint_step = 0.05

        # 单次笛卡尔增量（米）
        # Cartesian increment per step (meters)
        self.cart_step = 0.01

        # 摇杆死区，防止微小抖动
        # Deadzone to avoid joystick noise
        self.deadzone = 0.5

        # 初始模式：笛卡尔控制
        # Initial control mode: Cartesian
        self.mode_cart = True

        # 上一次模式切换按钮状态（用于防抖）
        # Last state of mode switch button (debounce)
        self.last_btn_mode = 0

        # 是否被外部节点强制切换模式
        # Whether mode is overridden externally
        self.external_mode_override = False

        # 夹爪最大力/范围参数
        # Gripper scaling parameter
        self.gripper_effort = 1.0

        # 夹爪当前绝对值
        # Current absolute gripper value
        self.grip_val = 0.0

        # 创建定时器，20 Hz 主循环
        # Create main loop timer at 20 Hz
        self.timer = self.create_timer(0.05, self.loop)

        # 发布初始控制模式
        # Publish initial control mode
        self.publish_mode()

    # ============================================================
    def publish_mode(self):
        """
        	发布当前控制模式到 /control_mode
        Publish current control mode to /control_mode
        """
        msg = String()
        msg.data = "cartesian" if self.mode_cart else "joint"
        self.pub_mode.publish(msg)

    # ============================================================
    def mode_callback(self, msg: String):
        """
        	接收外部节点发来的模式切换指令
        Callback for external control mode override
        """
        mode = msg.data.strip().lower()

        if mode == "cartesian" and not self.mode_cart:
            self.mode_cart = True
            self.external_mode_override = True
            self.get_logger().info(
                "📡 外部切换到【笛卡尔模式】 | Switched to CARTESIAN externally")

        elif mode == "joint" and self.mode_cart:
            self.mode_cart = False
            self.external_mode_override = True
            self.get_logger().info(
                "📡 外部切换到【关节模式】 | Switched to JOINT externally")

    # ============================================================
    def loop(self):
        """
        	主循环：轮询手柄、切换模式、执行控制
        Main loop: poll joystick, switch mode, execute control
        """
        pygame.event.pump()

        # 读取模式切换按钮（如 Start 键）
        # Read mode toggle button (e.g. START)
        btn_mode = self.joy.get_button(7)

        # 检测上升沿，防止连续切换
        # Rising-edge detection to avoid repeated toggling
        if btn_mode and not self.last_btn_mode:
            self.mode_cart = not self.mode_cart
            self.get_logger().info(
                f"🔁 切换模式 | Mode switched to: "
                f"{'CARTESIAN' if self.mode_cart else 'JOINT'}")
            self.publish_mode()
            self.external_mode_override = False

        self.last_btn_mode = btn_mode

        # 根据当前模式执行对应控制
        # Execute active control mode
        if self.mode_cart:
            self.handle_cartesian()
        else:
            self.handle_joint_delta()

    # ============================================================
    def handle_joint_delta(self):
        """
        	关节空间增量控制（Δq）
        Joint-space incremental control (Δq)
        """
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        hats = [self.joy.get_hat(i) for i in range(self.joy.get_numhats())]
        buttons = [self.joy.get_button(i)
                   for i in range(self.joy.get_numbuttons())]

        # 六个关节的增量 Δq₁…Δq₆
        # Incremental joint vector Δq₁…Δq₆
        dq = [0.0] * 6

        # Joint 1：左摇杆 X
        # Joint 1 controlled by left joystick X-axis
        if abs(axes[0]) > self.deadzone:
            dq[0] = self.joint_step * (-1 if axes[0] > 0 else 1)

        # Joint 2 & 3：方向键
        # Joint 2 & 3 controlled by D-pad
        hat_x, hat_y = hats[0]
        if hat_x != 0:
            dq[1] = self.joint_step * hat_x
        if hat_y != 0:
            dq[2] = self.joint_step * hat_y

        # Joint 4：右摇杆 Y
        # Joint 4 controlled by right joystick Y-axis
        if abs(axes[3]) > self.deadzone:
            dq[3] = self.joint_step * (-1 if axes[3] > 0 else 1)

        # Joint 5：按钮控制
        # Joint 5 controlled by buttons
        if buttons[3]:
            dq[4] = self.joint_step
        elif buttons[1]:
            dq[4] = -self.joint_step

        # Joint 6：按钮控制
        # Joint 6 controlled by buttons
        if buttons[4]:
            dq[5] = self.joint_step
        elif buttons[0]:
            dq[5] = -self.joint_step

        # 夹爪：绝对控制
        # Gripper absolute control
        grip_axis = axes[5]
        self.grip_val = 0.35 * (-grip_axis + 1) / 2.0 if grip_axis <= 0 else 0.0
        self.pub_grip.publish(Float64(data=self.grip_val))

        # 构造 JointState 消息
        # Build JointState message
        msg = JointState()
        msg.name = [f'joint{i+1}' for i in range(8)]
        msg.position = dq + [self.grip_val, 0.0]
        msg.velocity = [0.0] * 8
        msg.effort = [0.0] * 8
        self.pub_joint.publish(msg)

    # ============================================================
    def handle_cartesian(self):
        """
        	笛卡尔空间增量控制（Δx, Δy, Δz）
        Cartesian incremental control (Δx, Δy, Δz)
        """
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        hat_x, hat_y = self.joy.get_hat(0)

        # 轴映射（可根据手柄型号调整）
        # Axis mapping (adjust for different controllers)
        axis_x = 0  # 左摇杆水平 | Left stick horizontal
        axis_z = 3  # 右摇杆垂直 | Right stick vertical

        dx = self.cart_step * (-1 if axes[axis_x] >
                               0 else 1) if abs(axes[axis_x]) > self.deadzone else 0.0
        dy = self.cart_step * hat_y if hat_y != 0 else 0.0
        dz = self.cart_step * (-1 if axes[axis_z] >
                               0 else 1) if abs(axes[axis_z]) > self.deadzone else 0.0

        if dx != 0.0 or dy != 0.0 or dz != 0.0:
            msg = Point(x=dx, y=dy, z=dz)
            self.pub_cart.publish(msg)


# ============================================================
def main(args=None):
    """
    ROS2 主入口
    ROS2 main entry point
    """
    rclpy.init(args=args)
    node = PiperTeleopGamepad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

