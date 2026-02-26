#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
import math

class HybridMotionController(Node):
    def __init__(self):
        super().__init__('hybrid_motion_controller')
        
        # 订阅cmd_vel话题
        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # 订阅IMU话题
        self.imu_subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # 发布速度命令给Quadruped Controller（处理腿部控制）
        self.quadruped_cmd_publisher = self.create_publisher(
            Twist, '/cmd_vel/smooth', 10)
        
        # 发布轮子速度控制
        self.wheel_publisher = self.create_publisher(
            Float64MultiArray, '/joint_group_velocity_controller/commands', 10)
        
        # 轮子关节名称（4个轮子）
        self.wheel_joint_names = [
            'FL_foot_joint', 'FR_foot_joint', 'RL_foot_joint', 'RR_foot_joint'
        ]
        
        # 轮子速度参数
        self.wheel_radius = 0.05   # 轮子半径（米）
        self.wheel_base = 0.3      # 轮子间距
        self.wheel_track = 0.2     # 轮子轴距
        
        # IMU数据
        self.pitch_angle = 0.0
        self.pitch_threshold = 90.0  # 上坡阈值（度）
        
        self.get_logger().info('混合运动控制器已启动')
    
    def imu_callback(self, msg):
        """处理IMU数据，提取pitch角度"""
        # 从四元数提取pitch角度
        quaternion = msg.orientation
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        
        # 转换四元数为欧拉角
        # t0 = +2.0 * (w * x + y * z)
        # t1 = +1.0 - 2.0 * (x * x + y * y)
        # roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        # 转换为角度
        self.pitch_angle = math.degrees(pitch_y)
        
    
    def cmd_vel_callback(self, msg):
        """处理速度命令，根据pitch角度决定控制策略"""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        # 检测是否上坡（pitch超过5度）
        self.get_logger().info(f'pitch_angle: {self.pitch_angle:.2f}')
        is_uphill = self.pitch_angle < -self.pitch_threshold
        
        if is_uphill:
            # 上坡模式：轮子和腿部同时前进
            self.get_logger().info(f'上坡模式: pitch={self.pitch_angle:.2f}度')
            
            # 发送完整的速度命令给Quadruped Controller（包括x方向）
            full_msg = Twist()
            full_msg.linear.x = linear_x
            full_msg.linear.y = linear_y
            full_msg.linear.z = 0.0
            full_msg.angular.x = 0.0
            full_msg.angular.y = 0.0
            full_msg.angular.z = angular_z
            
            self.quadruped_cmd_publisher.publish(full_msg)
            
            # 控制轮子转动速度（使用完整的速度命令）
            self.control_wheels(linear_x * 0.5, linear_y, angular_z)
        else:
            # 平地模式：轮子控制x方向，腿部控制y和z方向
            # 创建过滤后的速度命令（去掉x方向速度）
            filtered_msg = Twist()
            filtered_msg.linear.x = 0.0  # 过滤掉x方向速度
            filtered_msg.linear.y = linear_y
            filtered_msg.linear.z = 0.0
            filtered_msg.angular.x = 0.0
            filtered_msg.angular.y = 0.0
            filtered_msg.angular.z = angular_z
            
            # 转发过滤后的速度命令给Quadruped Controller（处理腿部控制）
            self.quadruped_cmd_publisher.publish(filtered_msg)
            
            # 控制轮子转动速度（使用完整的速度命令）
            self.control_wheels(linear_x, linear_y, angular_z)
    
    def control_wheels(self, linear_x, linear_y, angular_z):
        """控制轮子转动速度"""
        # 计算每个轮子的速度（差速驱动模型）
        wheel_speeds = self.calculate_wheel_speeds(linear_x, linear_y, angular_z)
        
        # 创建轮子速度消息
        wheel_msg = Float64MultiArray()
        wheel_msg.data = wheel_speeds
        
        # 发布轮子控制
        self.wheel_publisher.publish(wheel_msg)
        
        # self.get_logger().info(f'轮子控制发布: {wheel_speeds}')
    
    def calculate_wheel_speeds(self, linear_x, linear_y, angular_z):
        """计算轮子速度（差速驱动）"""
        # 差速驱动模型
        v_left = linear_x - angular_z * self.wheel_base / 2
        v_right = linear_x + angular_z * self.wheel_base / 2
        
        # 考虑横向运动（全向轮模型简化）
        v_front = linear_y + angular_z * self.wheel_track / 2
        v_rear = linear_y - angular_z * self.wheel_track / 2
        
        # 转换为轮子角速度（rad/s）
        wheel_speeds = [
            (v_left + v_front) / self.wheel_radius,   # FL
            (v_right + v_front) / self.wheel_radius,  # FR
            (v_left + v_rear) / self.wheel_radius,    # RL
            (v_right + v_rear) / self.wheel_radius    # RR
        ]
        
        return wheel_speeds

def main(args=None):
    rclpy.init(args=args)
    
    hybrid_controller = HybridMotionController()
    
    try:
        rclpy.spin(hybrid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        hybrid_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
