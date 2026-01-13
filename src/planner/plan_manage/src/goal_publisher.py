#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import threading
import math

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        
        # 参数
        self.declare_parameter('goal_x', 10.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_z', 0.5)
        self.declare_parameter('publish_rate', 1.0)
        
        # 获取参数
        goal_x = self.get_parameter('goal_x').value
        goal_y = self.get_parameter('goal_y').value
        goal_z = self.get_parameter('goal_z').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # 创建发布器
        self.goal_publisher = self.create_publisher(
            PointStamped,
            'goal_point',
            10
        )
        
        # 设置目标点
        self.goal_point = PointStamped()
        self.goal_point.header.frame_id = 'world'
        self.goal_point.point.x = goal_x
        self.goal_point.point.y = goal_y
        self.goal_point.point.z = goal_z
        
        # 创建定时器发布目标点
        self.timer = self.create_timer(1.0/publish_rate, self.publish_goal)
        
        self.get_logger().info(f'目标点发布器已启动，目标位置: ({goal_x}, {goal_y}, {goal_z})')
    
    def publish_goal(self):
        """发布目标点"""
        self.goal_point.header.stamp = self.get_clock().now().to_msg()
        self.goal_publisher.publish(self.goal_point)
        self.get_logger().debug(f'发布目标点: ({self.goal_point.point.x}, {self.goal_point.point.y}, {self.goal_point.point.z})')
    
    def set_goal(self, x, y, z):
        """设置新的目标点"""
        self.goal_point.point.x = x
        self.goal_point.point.y = y
        self.goal_point.point.z = z
        self.get_logger().info(f'设置新目标点: ({x}, {y}, {z})')

def main(args=None):
    rclpy.init(args=args)
    
    node = GoalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()