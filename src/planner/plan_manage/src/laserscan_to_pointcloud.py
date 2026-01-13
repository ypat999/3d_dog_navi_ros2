#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point32
import math

class LaserScanToPointCloud(Node):
    def __init__(self):
        super().__init__('laserscan_to_pointcloud')
        
        # 参数
        self.declare_parameter('target_frame', 'lidar')
        self.declare_parameter('min_height', 0.0)
        self.declare_parameter('max_height', 1.0)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 20.0)
        
        # 订阅激光雷达数据
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/scan',
            self.laser_callback,
            10
        )
        
        # 发布点云数据
        self.publisher = self.create_publisher(
            PointCloud2,
            'cloud',
            10
        )
        
        self.get_logger().info('LaserScan to PointCloud2转换节点已启动')
    
    def laser_callback(self, msg):
        """将LaserScan数据转换为PointCloud2"""
        
        # 获取参数
        target_frame = self.get_parameter('target_frame').value
        min_height = self.get_parameter('min_height').value
        max_height = self.get_parameter('max_height').value
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value
        
        # 创建点云数据
        points = []
        
        # 遍历激光雷达数据
        for i, range_val in enumerate(msg.ranges):
            # 过滤无效数据
            if range_val < range_min or range_val > range_max or math.isinf(range_val) or math.isnan(range_val):
                continue
            
            # 计算角度
            angle = msg.angle_min + i * msg.angle_increment
            
            # 转换为笛卡尔坐标
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            z = 0.0  # 2D激光雷达的z坐标为0
            
            # 添加点
            points.append([x, y, z])
        
        # 创建PointCloud2消息
        header = msg.header
        header.frame_id = target_frame
        
        # 创建点云消息
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        
        # 发布点云
        self.publisher.publish(cloud_msg)
        
        # 调试信息
        self.get_logger().debug(f'转换了 {len(points)} 个点')

def main(args=None):
    rclpy.init(args=args)
    
    node = LaserScanToPointCloud()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()