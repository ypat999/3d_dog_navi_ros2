import sys
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from .utils import traj2ros
from .planner_wrapper import TomogramPlanner
from .config import Config


class PCTPlannerNode(Node):
    def __init__(self):
        super().__init__('pct_planner_node')
        
        # 声明参数
        self.declare_parameter('scene', 'Spiral')
        self.declare_parameter('publish_rate', 10)
        
        # 获取参数
        self.scene = self.get_parameter('scene').value
        
        # 创建发布者
        qos_profile = QoSProfile(depth=10)
        self.path_pub = self.create_publisher(Path, '/pct_path', qos_profile)
        
        # 创建订阅者 - 接收目标点
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',  # 标准目标话题
            self.goal_callback,
            qos_profile
        )
        
        # 创建订阅者 - 接收RViz2的2D Nav Goal
        self.nav_goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.nav_goal_callback,
            qos_profile
        )
        
        # 初始化规划器
        cfg = Config()
        self.planner = TomogramPlanner(cfg)
        
        # 尝试加载tomogram
        try:
            tomo_file = 'building2_9'  # 默认文件
            if self.scene == 'Spiral':
                tomo_file = 'spiral0_3_2'
            elif self.scene == 'Plaza':
                tomo_file = 'plaza'
                
            self.planner.loadTomogram(tomo_file)
            self.get_logger().info(f'Loaded tomogram for scene: {self.scene}')
        except Exception as e:
            self.get_logger().error(f'Failed to load tomogram: {e}')
        
        # 初始化位置
        self.start_pos = np.array([-5.5, 6.0, 0.5], dtype=np.float32)
        self.end_pos = np.array([2.0, -3.0, 4.5], dtype=np.float32)
        
        # 定时器用于规划
        self.plan_timer = self.create_timer(0.5, self.plan_callback)
        
        # 上次规划的位置，用于避免重复规划
        self.last_planned_start_pos = None
        self.last_planned_end_pos = None
        
        self.get_logger().info(f'PCT Planner Node initialized for scene: {self.scene}')

    def goal_callback(self, msg):
        """处理目标点回调"""
        self.end_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y, 
            msg.pose.position.z
        ], dtype=np.float32)
        self.get_logger().info(f'New goal received: {self.end_pos}')

    def nav_goal_callback(self, msg):
        """处理RViz2 2D Nav Goal回调"""
        self.end_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y, 
            msg.pose.position.z
        ], dtype=np.float32)
        self.get_logger().info(f'New nav goal received: {self.end_pos}')

    def plan_callback(self):
        """定时规划回调函数"""
        if not hasattr(self, 'planner'):
            return
            
        # 检查位置是否发生变化（考虑浮点数精度）
        position_changed = True
        if self.last_planned_start_pos is not None and self.last_planned_end_pos is not None:
            if (np.linalg.norm(self.end_pos - self.last_planned_end_pos) < 0.01 and 
                np.linalg.norm(self.start_pos - self.last_planned_start_pos) < 0.01):
                position_changed = False
        
        # 只有位置变化时才执行规划
        if position_changed:
            try:
                # 获取当前机器人位置，这里使用固定起始点，实际应用中应从定位系统获取
                current_pos = self.start_pos  # 在实际应用中，这里应该从定位话题获取当前位
                traj_3d = self.planner.plan(current_pos, self.end_pos)
                if traj_3d is not None:
                    path_msg = traj2ros(traj_3d)
                    # 设置时间戳
                    path_msg.header.stamp = self.get_clock().now().to_msg()
                    self.path_pub.publish(path_msg)
                    self.get_logger().info(f"Published path: start_pos:{current_pos}, end_pos:{self.end_pos}")
                    
                    # 更新上次规划位置
                    self.last_planned_start_pos = current_pos.copy()
                    self.last_planned_end_pos = self.end_pos.copy()
            except Exception as e:
                self.get_logger().error(f"Path planning failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PCTPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()