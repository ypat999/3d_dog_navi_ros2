import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import std_msgs.msg


class TomographyNode(Node):
    def __init__(self):
        super().__init__('tomography_node')
        
        # 声明参数
        self.declare_parameter('scene', 'Spiral')
        self.declare_parameter('voxel_size', 0.1)
        
        # 获取参数
        self.scene = self.get_parameter('scene').value
        self.voxel_size = self.get_parameter('voxel_size').value
        
        # 创建点云订阅者
        qos_profile = QoSProfile(depth=10)
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/pointcloud_input',  # 输入点云话题
            self.pc_callback,
            qos_profile
        )
        
        # 创建处理完成标志
        self.processed = False
        
        self.get_logger().info(f'Tomography Node initialized for scene: {self.scene}')

    def pc_callback(self, msg):
        """处理输入点云"""
        if self.processed:
            return  # 只处理一次
        
        # 在实际实现中，这里会处理点云并生成tomogram
        # 由于这是复杂的算法，这里仅作为框架
        
        self.get_logger().info('Processing point cloud for tomography...')
        
        # 标记为已处理
        self.processed = True


def main(args=None):
    rclpy.init(args=args)
    node = TomographyNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()