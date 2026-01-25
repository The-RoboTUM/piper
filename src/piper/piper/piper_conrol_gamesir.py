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
        # 初始化 ROS2 节点，节点名为 piper_conrol_gamesir
        # Initialize ROS2 node with name "piper_conrol_gamesir"
        super().__init__('piper_conrol_gamesir')

        # ============================================================
        # 📡 ROS2 发布器（Publishers）
        # ROS2 Publishers
        # ============================================================

        # 发布笛卡尔空间位移增量 Δx, Δy, Δz（给 IK / Servo 使用）
        # Publish Cartesian delta motion (Δx, Δy, Δz) for IK / Servo
        self.pub_cart = self.create_publisher(Point, '/piper_cartesian_delta', 10)

        # 发布关节指令（JointState，可为增量或绝对）
        # Publish joint commands using JointState
        self.pub_joint = self.create_publisher(JointState, '/joint_commands_joi', 10)

        # 发布夹爪控制指令（绝对值）
        # Publish gripper command (absolute value)
        self.pub_grip = self.create_publisher(Float64, '/gripper_command', 10)

        # 发布控制模式（cartesian / joint）
        # Publish control mode ("cartesian" or "joint")
        self.pub_mode = self.create_publisher(String, '/control_mode', 10)

        # ============================================================
        # 📥 ROS2 订阅器（Subscribers）
        # ROS2 Subscribers
        # ============================================================

        # 订阅控制模式（允许外部节点切换模式）
        # Subscribe to external control mode commands
        self.sub_mode = self.create_subscription(
            String, '/control_mode', self.mode_callback, 10
        )

        # 订阅当前机器人关节状态（用于 joint ABS 模式）
        # Subscribe to current joint states (for absolute joint control)
        self.sub_joint_states = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # ============================================================
        # 🎮 初始化 pygame 手柄系统
        # Initialize pygame and joystick system
        # ============================================================

        pygame.init()
        pygame.joystick.init()

        # 如果没有检测到手柄，直接退出
        # Exit if no gamepad is detected
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("❌ 未检测到手柄 | No gamepad detected")
            exit()

        # 使用第一个手柄
        # Use the first detected joystick
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        self.get_logger().info(f"🎮 已检测到手柄 | Gamepad detected: {self.joy.get_name()}")

        # ============================================================
        # ⚙️ 控制参数（Parameters）
        # Control parameters
        # ============================================================

        # 单次关节步进（弧度）
        # Joint step size (radians per update)
        self.joint_step = 0.05

        # 单次笛卡尔步进（米）
        # Cartesian step size (meters per update)
        self.cart_step = 0.01

        # 手柄死区（防止抖动）
        # Deadzone to avoid joystick noise
        self.deadzone = 0.5

        # 初始模式：False = 关节模式
        # Initial mode: False = joint mode
        self.mode_cart = False

        # 记录上一次“模式切换按钮”状态（用于防抖）
        # Store previous mode button state (debounce)
        self.last_btn_mode = 0

        # 是否被外部节点强制切换模式
        # Flag indicating external mode override
        self.external_mode_override = False

        # 当前夹爪值（绝对）
        # Current gripper value (absolute)
        self.grip_val = 0.0

        # 当前关节状态缓存（8 关节：6 机械臂 + gripper + dummy）
        # Cache of current joint positions
        self.current_joints = [0.0] * 8

        # 关节名称列表
        # Joint name list
        self.joint_names = [f'joint{i+1}' for i in range(8)]

        # ============================================================
        # ⏱ 定时器（Timers）
        # Timers
        # ============================================================

        # 主控制循环，20 Hz
        # Main control loop at 20 Hz
        self.timer = self.create_timer(0.05, self.loop)

        # 发布初始模式
        # Publish initial control mode
        self.publish_mode()

        # 启动后 1 秒发送一次 HOME 位姿
        # Send HOME position once after startup
        self.home_timer = self.create_timer(1.0, self._trigger_home_once)

    # ============================================================
    def _trigger_home_once(self):
        """
        	启动后只执行一次 HOME 动作，然后取消定时器
        Trigger HOME motion once, then cancel timer
        """
        self.send_home_position_once()
        self.home_timer.cancel()

    # ============================================================
    def send_home_position_once(self):
        """
        	启动时发送 HOME 位姿（所有关节 = 0，夹爪关闭）
        Send HOME position at startup (all joints zero, gripper closed)
        """
        self.mode_cart = False
        self.publish_mode()

        msg = JointState()
        msg.name = self.joint_names
        msg.position = [0.0] * 8
        msg.velocity = [0.0] * 8
        msg.effort = [0.0] * 8

        self.pub_joint.publish(msg)
        self.pub_grip.publish(Float64(data=0.0))

        self.get_logger().info("🏠 已发送 HOME 位姿 | HOME position sent")

    # ============================================================
    def go_home_and_exit(self):
        """
        	节点退出前：回 HOME + 关闭夹爪
        Before shutdown: go HOME and close gripper
        """
        self.get_logger().info("🛑 正在退出，发送 HOME 位姿 | Exiting, sending HOME")

        self.mode_cart = False
        self.publish_mode()

        msg = JointState()
        msg.name = self.joint_names
        msg.position = [0.0] * 8
        msg.velocity = [0.0] * 8
        msg.effort = [0.0] * 8

        self.pub_joint.publish(msg)
        self.pub_grip.publish(Float64(data=0.0))

        rclpy.spin_once(self, timeout_sec=0.5)

    # ============================================================
    def joint_state_callback(self, msg: JointState):
        """
        	接收当前机器人关节状态
        Receive current joint states from robot
        """
        if len(msg.position) >= len(self.current_joints):
            self.current_joints = list(msg.position[:8])

    # ============================================================
    def publish_mode(self):
        """
        	发布当前控制模式
        Publish current control mode
        """
        msg = String()
        msg.data = "cartesian" if self.mode_cart else "joint"
        self.pub_mode.publish(msg)

    # ============================================================
    def mode_callback(self, msg: String):
        """
        	接收外部控制模式切换
        Receive external mode switch command
        """
        mode = msg.data.strip().lower()
        if mode == "cartesian" and not self.mode_cart:
            self.mode_cart = True
            self.external_mode_override = True
            self.get_logger().info("🛰️ 外部切换为 CARTESIAN 模式 | Switched to CARTESIAN")
        elif mode == "joint" and self.mode_cart:
            self.mode_cart = False
            self.external_mode_override = True
            self.get_logger().info("🛰️ 外部切换为 JOINT 模式 | Switched to JOINT")

    # ============================================================
    def loop(self):
        """
        	主控制循环（读取手柄 → 发布指令）
        Main control loop (read joystick → publish commands)
        """
        pygame.event.pump()
        btn_mode = self.joy.get_button(7)

        # 模式切换按钮（防抖）
        # Mode toggle button with debounce
        if btn_mode and not self.last_btn_mode:
            self.mode_cart = not self.mode_cart
            self.publish_mode()
            self.external_mode_override = False
        self.last_btn_mode = btn_mode

        # 根据当前模式执行控制
        # Execute control according to current mode
        if self.mode_cart:
            self.handle_cartesian()
        else:
            self.handle_joint_delta()

    # ============================================================
    def handle_cartesian(self):
        """
        	笛卡尔增量控制模式（Δx, Δy, Δz）
        Cartesian incremental control mode
        """
        axes = [self.joy.get_axis(i) for i in range(self.joy.get_numaxes())]
        hat_x, hat_y = self.joy.get_hat(0)

        # 左摇杆 X → Δx
        # Left joystick horizontal → Δx
        axis_x = 0

        # 右摇杆 Y → Δz
        # Right joystick vertical → Δz
        axis_z = 3

        dx = self.cart_step * (-1 if axes[axis_x] > 0 else 1) if abs(axes[axis_x]) > self.deadzone else 0.0
        dy = self.cart_step * hat_y if hat_y != 0 else 0.0
        dz = self.cart_step * (-1 if axes[axis_z] > 0 else 1) if abs(axes[axis_z]) > self.deadzone else 0.0

        if dx or dy or dz:
            self.pub_cart.publish(Point(x=dx, y=dy, z=dz))
            os.system('clear')
            print(f"[CARTESIAN Δ] Δx={dx:+.3f} Δy={dy:+.3f} Δz={dz:+.3f}")


# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = PiperTeleopGamepad()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.go_home_and_exit()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

