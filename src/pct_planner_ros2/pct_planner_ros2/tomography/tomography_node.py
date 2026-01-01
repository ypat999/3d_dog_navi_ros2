#!/usr/bin/env python3
import os
import sys
import time
import pickle
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from .tomogram import Tomogram

# 添加路径
sys.path.append('../')
from .config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from .config import Config

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomographyNode(Node):
    def __init__(self, cfg, scene_cfg):
        super().__init__('tomography_node')
        
        self.export_dir = rsg_root + cfg.map.export_dir
        self.pcd_file = scene_cfg.pcd.file_name
        self.resolution = scene_cfg.map.resolution
        self.ground_h = scene_cfg.map.ground_h
        self.slice_dh = scene_cfg.map.slice_dh

        self.center = np.zeros(2, dtype=np.float32)
        self.tomogram = Tomogram(scene_cfg)
        
        # 初始化QoS
        qos_profile = QoSProfile(depth=10)
        
        # 初始化ROS2发布者
        self.init_ros2_publishers(qos_profile)
        
        # 加载并处理点云
        points = self.loadPCD(self.pcd_file)
        self.process(points)

    def init_ros2_publishers(self, qos_profile):
        """初始化ROS2发布者"""
        self.map_frame = "map"  # ROS2中的默认地图帧
        
        # 点云发布者
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, 
            '/global_points',  # cfg.ros.pointcloud_topic
            qos_profile
        )

        # 层发布者
        self.layer_G_pub_list = []
        self.layer_C_pub_list = []
        
        for i in range(20):  # 假设最多20层，实际会根据数据调整
            layer_G_pub = self.create_publisher(
                PointCloud2, 
                f'/layer_G_{i}',  # cfg.ros.layer_G_topic + str(i)
                qos_profile
            )
            self.layer_G_pub_list.append(layer_G_pub)
            
            layer_C_pub = self.create_publisher(
                PointCloud2, 
                f'/layer_C_{i}',  # cfg.ros.layer_C_topic + str(i)
                qos_profile
            )
            self.layer_C_pub_list.append(layer_C_pub)

        # 断层图发布者
        self.tomogram_pub = self.create_publisher(
            PointCloud2, 
            '/tomogram',  # cfg.ros.tomogram_topic
            qos_profile
        )

    def loadPCD(self, pcd_file):
        """加载PCD文件"""
        pcd_path = os.path.join(rsg_root, "rsc/pcd", pcd_file)
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points).astype(np.float32)
        self.get_logger().info(f"PCD points: {points.shape[0]}")

        if points.shape[1] > 3:
            points = points[:, :3]
        self.points_max = np.max(points, axis=0)
        self.points_min = np.min(points, axis=0)           
        self.points_min[-1] = self.ground_h
        
        self.map_dim_x = int(np.ceil((self.points_max[0] - self.points_min[0]) / self.resolution)) + 4
        self.map_dim_y = int(np.ceil((self.points_max[1] - self.points_min[1]) / self.resolution)) + 4
        n_slice_init = int(np.ceil((self.points_max[2] - self.points_min[2]) / self.slice_dh))
        self.center = (self.points_max[:2] + self.points_min[:2]) / 2
        self.slice_h0 = self.points_min[-1] + self.slice_dh
        self.tomogram.initMappingEnv(self.center, self.map_dim_x, self.map_dim_y, n_slice_init, self.slice_h0)

        self.get_logger().info(f"Map center: [{self.center[0]:.2f}, {self.center[1]:.2f}]")
        self.get_logger().info(f"Dim_x: {self.map_dim_x}")
        self.get_logger().info(f"Dim_y: {self.map_dim_y}")
        self.get_logger().info(f"Num slices init: {n_slice_init}")

        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

        return points
        
    def process(self, points):        
        """处理点云数据"""
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0
        n_repeat = 10

        """ 
        GPU time benchmark, where CUDA events are synchronized for correct time measurement.
        The function is repeatedly run for n_repeat times to calculate the average processing time of each modules.
        The time of the first warm-up run is excluded to reduce timing fluctuation and exclude the overhead in initial invocations.
        See https://docs.cupy.dev/en/stable/user_guide/performance.html for more details
        """
        for i in range(n_repeat + 1):
            t_start = time.time()
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = self.tomogram.point2map(points)

            if i > 0:
                t_map += t_gpu['t_map']
                t_trav += t_gpu['t_trav']
                t_simp += t_gpu['t_simp']
                t_all += (time.time() - t_start) * 1e3

        self.get_logger().info(f"Num slices simp: {layers_g.shape[0]}")
        self.get_logger().info(f"Num repeats (for benchmarking only): {n_repeat}")
        self.get_logger().info(f" -- avg t_map  (ms): {t_map / n_repeat}")
        self.get_logger().info(f" -- avg t_trav (ms): {t_trav / n_repeat}")
        self.get_logger().info(f" -- avg t_simp (ms): {t_simp / n_repeat}")
        self.get_logger().info(f" -- avg t_all  (ms): {t_all / n_repeat}")

        self.n_slice = layers_g.shape[0]

        map_file = os.path.splitext(self.pcd_file)[0]
        self.exportTomogram(np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)), map_file)

        # 初始化ROS2发布者（已经在__init__中完成）
        self.publishPoints(points)
        self.publishLayers(self.layer_G_pub_list[:self.n_slice], layers_g, layers_t)
        self.publishLayers(self.layer_C_pub_list[:self.n_slice], layers_c, None)
        self.publishTomogram(layers_g, layers_t)

    def exportTomogram(self, tomogram, map_file):        
        """导出断层图"""
        data_dict = {
            'data': tomogram.astype(np.float16),
            'resolution': self.resolution,
            'center': self.center,
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
        }
        file_name = map_file + '.pickle'
        full_path = os.path.join(self.export_dir, file_name)
        
        # 确保目录存在
        os.makedirs(os.path.dirname(full_path), exist_ok=True)
        
        with open(full_path, 'wb') as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        self.get_logger().info(f"Tomogram exported: {file_name}")

    def publishPoints(self, points):
        """发布点云数据"""
        header = Header()
        header.frame_id = self.map_frame

        # 创建PointCloud2消息
        point_msg = pc2.create_cloud_xyz32(header, points.astype(np.float32))
        self.pointcloud_pub.publish(point_msg)

    def publishLayers(self, pub_list, layers, color=None):
        """发布层数据"""
        for i, layer in enumerate(layers):
            if i < len(pub_list):
                header = Header()
                header.frame_id = self.map_frame
                
                # 为每层创建点云
                points = self.create_layer_points(layer, color)
                point_msg = pc2.create_cloud_xyz32(header, points)
                pub_list[i].publish(point_msg)

    def create_layer_points(self, layer, color):
        """为层数据创建点云"""
        # 简化实现 - 实际应用中需要根据具体数据格式调整
        h, w = layer.shape
        points = []
        for y in range(h):
            for x in range(w):
                if layer[y, x] > 0:  # 假设非零值表示有效点
                    z = 0  # 默认高度，实际应用中应根据层索引设置
                    points.append([x * self.resolution, y * self.resolution, z])
        return np.array(points, dtype=np.float32)

    def publishTomogram(self, layers_g, layers_t):
        """发布断层图"""
        header = Header()
        header.frame_id = self.map_frame
        
        # 创建断层图的点云表示
        # 这里简化处理，实际应用中需要根据数据格式创建合适的点云
        point_msg = pc2.create_cloud_xyz32(header, np.array([[0, 0, 0]], dtype=np.float32))
        self.tomogram_pub.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    
    # 需要加载配置
    cfg = Config()
    # 注意：这里需要根据场景参数加载，实际使用时需要传递场景参数
    
    # 创建节点（这里简化，实际需要加载场景配置）
    node = TomographyNode(cfg, cfg.scene)  # 实际需要正确的场景配置
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()