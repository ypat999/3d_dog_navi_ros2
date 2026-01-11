#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class HybridMotionController(Node):
    def __init__(self):
        super().__init__('hybrid_motion_controller')
        
        # 订阅cmd_vel话题
        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
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
        
        self.get_logger().info('混合运动控制器已启动')
    
    def cmd_vel_callback(self, msg):
        """处理速度命令，转发给Quadruped Controller控制腿部，同时控制轮子"""
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        self.get_logger().info(f'收到速度命令: linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}')
        
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
        
        self.get_logger().info(f'转发给Quadruped Controller: linear_y={linear_y}, angular_z={angular_z}')
        
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
        
        self.get_logger().info(f'轮子控制发布: {wheel_speeds}')
    
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