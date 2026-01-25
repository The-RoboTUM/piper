#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np
import math
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class PiperCartesianController(Node):
    def __init__(self):
        super().__init__('piper_cartesian_controller')
        
        # 6D pose: [x, y, z, roll, pitch, yaw]
        self.current_pose = np.zeros(6)
        self.pose_initialized = True
        self.current_pose[:3] = np.array([0.5, 0.0, 0.15])  # safe starting EE pose
        self.gripper_hold = 0.002
        
        self.mode = self.declare_parameter('mode', 'hardware').value  # 'gazebo' or 'hardware'
        if self.mode == 'hardware': #Implement in real robot mode
    	    # Option 1 (matches their launch remap):
    	    # self.hw_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Option 2 (cleaner; use if you remove their 5):
            self.hw_pub = self.create_publisher(JointState, '/joint_ctrl_single', 10)

        else: #Implement in Gazebo
            # your existing gazebo publisher:
            self.arm_cmd_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
	
        # AgileX Piper 机械臂的 DH 参数（单位：米）
        #     这些参数定义了每个关节之间的几何关系，用于正/逆运动学计算
        # Denavit–Hartenberg (DH) parameters for AgileX Piper (meters)
        #     Used to describe kinematic chain for FK / IK computation
        self.dh_params = [
            # 关节 1：底座旋转关节（Base rotation）
            # Joint 1: Base revolute joint
            {'theta': 0, 'd': 0.123, 'a': 0.000, 'alpha': -math.pi/2},

            # 关节 2：肩部关节（Shoulder）
            # Joint 2: Shoulder joint
            {'theta': -177.22/180*math.pi, 'd': 0.000, 'a': 0.28505, 'alpha': 0},

            # 关节 3：肘部关节（Elbow）
            # Joint 3: Elbow joint
            {'theta': -102.78/180*math.pi, 'd': 0.000, 'a': -0.02198, 'alpha': math.pi/2},

            # 关节 4：腕部关节 1（Wrist 1）
            # Joint 4: Wrist joint 1
            {'theta': 0, 'd': 0.25065, 'a': 0, 'alpha': -math.pi/2},

            # 关节 5：腕部关节 2（Wrist 2）
            # Joint 5: Wrist joint 2
            {'theta': 0, 'd': 0.000, 'a': 0.000, 'alpha': math.pi/2},

            # 关节 6：腕部关节 3（末端法兰）
            # Joint 6: Wrist joint 3 / flange
            {'theta': 0, 'd': 0.091, 'a': 0.000, 'alpha': 0}
        ]
        
        # 当前机械臂关节状态（包含 8 个关节）
        #     前 6 个用于运动学计算，后 2 个通常是夹爪或附加轴
        # Current joint state (8 joints total)
        self.current_joint_positions = [0.0] * 8

        # 关节名称（必须与 URDF / joint_states topic 一致）
        # Joint names (must match URDF and /joint_states)
        self.joint_names = [ #TEMPORARILY REMOVING JOINT 7 AND 8
            'joint1','joint2','joint3',
            'joint4','joint5','joint6',
            'joint7','joint8'
        ]
        
        self.arm_joint_names = [
    	    'joint1', 'joint2', 'joint3',
            'joint4', 'joint5', 'joint6'
	]
        
        # 机械臂 HOME 位姿（启动时使用）, NOT USED YET!!!!!!!!!!!!!!!!!!!!
        # Home joint configuration
        #############self.home_position = [0.0] * 8
        
        # 发布关节命令（位置控制）
        #     ⚠️ 注意：这是直接往 /joint_states 发 position
        # Publisher for joint position commands
        """ COMMENTED OUT TEMPORARILY BY ALAN CHUAH
        self.joint_publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        """
        
        #WORKAROUND
        self.arm_cmd_pub = self.create_publisher(
            JointTrajectory, 
            '/arm_controller/joint_trajectory', 
            10
        )
        
        # 接收“绝对”笛卡尔目标（x, y, z）
        # Subscriber for absolute Cartesian target
        self.cartesian_subscriber = self.create_subscription(
            Point,
            '/piper_cartesian_target',
            self.cartesian_target_callback,
            10
        )
        
        # 接收“相对”笛卡尔位移（Δx, Δy, Δz）
        # Subscriber for Cartesian delta commands
        self.cartesian_delta_subscriber = self.create_subscription(
            Point,
            '/piper_cartesian_delta',
            self.cartesian_delta_callback,
            10
        )
        
        # 监听真实关节状态（用于 IK 初始值）
        # Subscriber for current joint states
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 启动后发送 HOME 位姿，确保初始状态一致
        # Send robot to home position on startup
        #self.publish_home_position()
        
        self.get_logger().info('🚀 Piper Cartesian Controller STARTED')
        self.get_logger().info('📡 Listening on /piper_cartesian_target & /piper_cartesian_delta')

    def send_joint_command_hardware(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1','joint2','joint3','joint4','joint5','joint6','joint7','joint8']
        msg.position = [
            float(q[0]),
            float(q[1]),
            float(q[2]),
            float(q[3]),
            float(q[4]),
            float(q[5]),
            self.gripper_hold,   # 🔑 gripper force/width
            0.0                 # 🔑 dummy joint (REQUIRED)
        ]
        # optional: include velocity if you want (driver uses it to set speed if nonzero)
        # msg.velocity = [0.0]*6
        self.hw_pub.publish(msg)

        
    def publish_home_position(self):
        """Publica posición HOME al iniciar"""
        time.sleep(1.0)
        self.send_joint_command(self.home_position)
        self.get_logger().info('🏠 Posición HOME enviada')
    
    def joint_state_callback(self, msg):
        try:
            for i, name in enumerate(self.joint_names):
                if name in msg.name:
                    idx = msg.name.index(name)
                    self.current_joint_positions[i] = msg.position[idx]

            # 🔑 Initialize Cartesian pose ONCE using FK
            if not self.pose_initialized:
                q6 = np.array(self.current_joint_positions[:6])
                ee_pos = self.cinematica_directa(q6)

                self.current_pose[:3] = ee_pos
                self.pose_initialized = True

                self.get_logger().warn(
                    f"📌 Cartesian pose initialized from FK: {ee_pos}"
                )

        except Exception as e:
            self.get_logger().error(f'Error updating joint states: {e}')
    
    def cartesian_target_callback(self, msg):
        """Callback para recibir coordenadas Cartesianas absolutas"""
        x, y, z = msg.x, msg.y, msg.z
        self.get_logger().info(f'🎯 Recibido target ABSOLUTO: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # Calcular cinemática inversa directa
        target_joints = self.calculate_inverse_kinematics(x, y, z)
        
        if target_joints is not None:
            self.send_joint_command(target_joints)
        else:
            self.get_logger().error('❌ No se pudo calcular la cinemática inversa')
    
    def cartesian_delta_callback(self, msg):
        if not self.pose_initialized:
            self.get_logger().warn("⏳ Waiting for joint states to initialize pose")
            return
        # 1. Read Cartesian delta
        delta = np.array([msg.x, msg.y, msg.z])

	# 2. Update internal pose (position only for now)
        self.current_pose[:3] += delta
        x, y, z = self.current_pose[:3]

        self.get_logger().info(
            f"Target EE position: x={x:.4f}, y={y:.4f}, z={z:.4f}"
        )
        
        # 3. 🔑 THIS IS WHERE THE FIRST LINE GOES (IK)
        q = self.calculate_inverse_kinematics(x, y, z)
        
        # 4. Safety check
        if q is None:
            self.get_logger().error("IK failed, skipping command")
            return   
        q[5] = -math.pi / 2

        self.get_logger().info(f"IK solution: {q}")

        # 5. 🔑 THIS IS WHERE THE SECOND LINE GOES (command)
        #self.send_joint_command(q)
        # 🔑 MODE SWITCH GOES HERE
        if self.mode == 'hardware':
            self.send_joint_command_hardware(q)
        else:
            self.send_joint_command(q)  # Gazebo
        
        """ TEMPORARY WORKAROUND FOR DUMB MOTION TESTING
        self.get_logger().error(
            f"🔥 DELTA RECEIVED: x={msg.x:.4f}, y={msg.y:.4f}, z={msg.z:.4f}"
        )
        
        traj = JointTrajectory()
        traj.joint_names = self.arm_joint_names

        pt = JointTrajectoryPoint()
        pt.positions = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]  # simple test pose
        pt.time_from_start = Duration(sec=1, nanosec=0)

        traj.points = [pt]
        self.arm_cmd_pub.publish(traj)
        """
    
    def matriz_transformacion_DH(self, theta, d, a, alpha):
        """根据 DH 参数生成单个关节的齐次变换矩阵"""
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0, math.sin(alpha), math.cos(alpha), d],
            [0, 0, 0, 1]
        ])
    
    def cinematica_directa(self, q):
        """正运动学（FK）
            	输入：6 维关节角 q
            	输出：末端执行器的笛卡尔位置 [x, y, z]
        """
        T = np.eye(4)
        
        for i in range(6):
            theta = self.dh_params[i]['theta'] + q[i]
            d = self.dh_params[i]['d']
            a = self.dh_params[i]['a'] 
            alpha = self.dh_params[i]['alpha']
            
            T_i = self.matriz_transformacion_DH(theta, d, a, alpha)
            T = T @ T_i
        
        return T[:3, 3]  # Retorna posición [x, y, z]
    
    def jacobiano_geometrico(self, q, delta=0.0001):
        """数值方式计算几何雅可比矩阵（仅位置部分）
        J = ∂x / ∂q
            	使用有限差分近似
        """
        jacobiano = np.zeros((3, 6))
        pos_actual = self.cinematica_directa(q)
        
        for i in range(6):
            q_pert = q.copy()
            q_pert[i] += delta
            pos_pert = self.cinematica_directa(q_pert)
            jacobiano[:, i] = (pos_pert - pos_actual) / delta
        
        return jacobiano
    
    def calculate_inverse_kinematics(self, x, y, z, max_iter=200, tol=1e-4):
        """数值迭代逆运动学（Jacobian pseudo-inverse）
            - 使用当前关节角作为初值（非常重要）
            - 只控制位置，不控制姿态
        """
        q_current = np.array(self.current_joint_positions[0:6])
        target_pos = np.array([x, y, z])
        
        self.get_logger().info(f'🔍 IK - Goal: [{x:.3f}, {y:.3f}, {z:.3f}]')
        
        # Algoritmo iterativo
        for iteration in range(max_iter):
            current_pos = self.cinematica_directa(q_current)
            error = target_pos - current_pos
            error_norm = np.linalg.norm(error)
            
            if error_norm < tol:
                self.get_logger().info(f'✅ IK - Converged in {iteration} iterations')
                result = q_current.tolist() + self.current_joint_positions[6:8]
                return result
            
            J = self.jacobiano_geometrico(q_current)
            
            try:
                J_pinv = np.linalg.pinv(J)
                dq = J_pinv @ error
                q_current = q_current + dq * 0.3  # Factor de ganancia reducido
                q_current = np.arctan2(np.sin(q_current), np.cos(q_current))
                
            except Exception as e:
                self.get_logger().error(f'❌ Error in IK calculation: {e}')
                return None
        
        self.get_logger().warning(f'⚠️ IK - Did not converge after {max_iter} iterations')
        return None

    def mover_cartesiano(self, dx, dy, dz, pasos=30):
        """笛卡尔空间的“增量平滑运动”
            - 把 Δx, Δy, Δz 拆成多步
            - 每一步都重新求 IK
        """
        self.get_logger().info(f'🔄 Smooth movement: Δx={dx:.3f}, Δy={dy:.3f}, Δz={dz:.3f}, steps={steps}')
        
        q_actual = np.array(self.current_joint_positions[0:6])
        pos_actual = self.cinematica_directa(q_actual)

        # Trayectoria lineal en el espacio cartesiano
        for i in range(1, steps + 1):
            # Nueva posición objetivo intermedia
            factor = i / steps
            pos_intermedia = pos_actual + np.array([dx, dy, dz]) * factor
            
            # IK numérica usando la configuración anterior como semilla
            q_obj = self.calculate_inverse_kinematics(
                pos_intermedia[0], 
                pos_intermedia[1], 
                pos_intermedia[2]
            )
            
            if q_obj is None:
                self.get_logger().warning('⚠️ Did not converge on intermediate step, stopping movement')
                break
            
            # Publicar posición intermedia
            self.send_joint_command(q_obj)
            time.sleep(0.03)  # Suaviza el movimiento (30ms increment steps)
            
            # Actualizar posición actual para el siguiente paso
            q_actual = np.array(q_obj[0:6])
        
        self.get_logger().info('✅ Smooth movement completed')
    """
    def send_joint_command(self, joint_positions):
        	#发送关节位置命令（含安全限制）
           # - 检查关节角是否超限
           # - 构造 JointState 并发布
        
        try:
            # Límites de seguridad para joints 1-6
            joint_limits = [
                [-3.14, 3.14], [-2.35, 2.35], [-3.14, 3.14],
                [-3.14, 3.14], [-3.14, 3.14], [-3.14, 3.14]
            ]
            
            # Verificar límites para joints 1-6
            for i in range(6):
                pos = joint_positions[i]
                if pos < joint_limits[i][0] or pos > joint_limits[i][1]:
                    self.get_logger().error(f'❌ Joint {i+1} fuera de límites: {pos:.3f} rad')
                    return
            
            # Crear mensaje JointState
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = [float(pos) for pos in joint_positions]
            
            # Publicar comando
            self.joint_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'❌ Error enviando comando: {e}')
    """
    
    def send_joint_command(self, joint_positions):
        # joint_positions is 8-length in your code; arm is first 6
        q6 = [float(x) for x in joint_positions[:6]]

        msg = JointTrajectory()
        msg.joint_names = self.arm_joint_names

        pt = JointTrajectoryPoint()
        pt.positions = q6
        pt.time_from_start = Duration(sec=1, nanosec=300_000_000)  # 1s move

        msg.points = [pt]
        self.arm_cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = PiperCartesianController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Controller turned off by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
