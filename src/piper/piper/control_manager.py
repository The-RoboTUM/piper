#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from enum import Enum
import threading

class ControlMode(Enum):
    JOINT = 1
    CARTESIAN = 2

class ControlManager(Node):
    def __init__(self):
        super().__init__('control_manager')
        
        # Publisher for joint states
        # 关节状态发布器
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Subscribers
        # 订阅器
        self.joint_cmd_sub = self.create_subscription(
            JointState, '/joint_commands_joi', self.joint_cmd_callback, 10)
        self.cartesian_cmd_sub = self.create_subscription(
            JointState, '/joint_commands_cart', self.cartesian_cmd_callback, 10)
        self.mode_sub = self.create_subscription(
            String, '/control_mode', self.mode_callback, 10)
        
        # Initial state – 8 joints in HOME position
        # 初始状态——8 个关节处于 HOME 位置
        self.current_mode = ControlMode.CARTESIAN
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # HOME position
        self.lock = threading.Lock()
        
        # Timer for periodic publishing (10 Hz)
        # 用于周期性发布的定时器（10 Hz）
        self.timer = self.create_timer(0.1, self.publish_joint_state)
        
        # Publish initial state immediately
        # 立即发布初始状态
        self.publish_joint_state()
        
        self.get_logger().info("Control Manager started – Initial state published") # 控制管理器已启动——初始状态已发布

    def mode_callback(self, msg):
    	# Handle control mode switching
        # 处理控制模式切换
        with self.lock:
            if msg.data.lower() == "cartesian":
                self.current_mode = ControlMode.CARTESIAN
                self.get_logger().info("Mode switched to: CARTESIAN") # 模式切换为：笛卡尔
            elif msg.data.lower() == "joint":
                self.current_mode = ControlMode.JOINT
                self.get_logger().info("Mode switched to: JOINT") # 模式切换为：关节
        
    def joint_cmd_callback(self, msg):
        """
        Handle commands in JOINT mode
        	处理关节控制模式下的命令
        """
        if self.current_mode != ControlMode.JOINT:
            return
            
        with self.lock:
            self.get_logger().info(f"Received JOINT command: {msg.position}") # 接收到关节命令
            
            # Teleoperation sends 7 joints, adapt to 8 joints
            # 遥操作发送 7 个关节，需要适配为 8 个关节
            for i, name in enumerate(msg.name):
                idx = self.get_joint_index(name)
                if idx is not None:
                    if idx < 6:  # 机械臂关节 1–6
                        self.current_joints[idx] = msg.position[i]
                    elif idx == 6:  # Gripper 夹爪
                        self.current_joints[6] = msg.position[i]  # joint7
                        self.current_joints[7] = -msg.position[i]  # joint8
            
            self.get_logger().info(f"Updated joints: {self.current_joints}") # 已更新关节状态

    def cartesian_cmd_callback(self, msg):
        """
        Handle commands in CARTESIAN mode
        	处理笛卡尔控制模式下的命令
        """
        if self.current_mode != ControlMode.CARTESIAN:
            return
            
        with self.lock:
            self.get_logger().info(f"Received CARTESIAN command: {msg.position}") # 接收到笛卡尔命令
            
            # Cartesian controller sends 8 joints
            # 笛卡尔控制器发送 8 个关节
            for i, name in enumerate(msg.name):
                idx = self.get_joint_index(name)
                if idx is not None and idx < len(self.current_joints):
                    self.current_joints[idx] = msg.position[i]
            
            self.get_logger().info(f"Updated joints: {self.current_joints}") # 已更新关节状态

    def publish_joint_state(self):
        """
        Publish current joint state to /joint_states
        	将当前关节状态发布到 /joint_states
        """
        with self.lock:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "world"
            msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']
            msg.position = self.current_joints.copy()
            
            self.joint_pub.publish(msg)
            self.get_logger().debug(f"Published joint state: {msg.position}", throttle_duration_sec=2.0) # 已发布关节状态（限频输出）

    def get_joint_index(self, name):
        """
        Map joint name to index
        	将关节名称映射为索引
        """
        mapping = {
            'joint1': 0, 'joint2': 1, 'joint3': 2,
            'joint4': 3, 'joint5': 4, 'joint6': 5,
            'joint7': 6, 'joint8': 7
        }
        return mapping.get(name, None)

def main(args=None):
    # Initialize ROS 2
    # 初始化 ROS 2
    rclpy.init(args=args)
    node = ControlManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
