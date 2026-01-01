#!/usr/bin/env python3
# Test script to verify the ROS2 PCT Planner package

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TestPCTPlannerNode(Node):
    def __init__(self):
        super().__init__('test_pct_planner_node')
        
        # 创建订阅者来监听路径消息
        self.path_sub = self.create_subscription(
            Path,
            '/pct_path',
            self.path_callback,
            10
        )
        
        self.path_received = False
        self.get_logger().info('Test node initialized, waiting for path messages...')

    def path_callback(self, msg):
        if not self.path_received:
            self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')
            if len(msg.poses) > 0:
                first_pose = msg.poses[0]
                last_pose = msg.poses[-1] if len(msg.poses) > 0 else first_pose
                self.get_logger().info(f'First waypoint: ({first_pose.pose.position.x:.2f}, {first_pose.pose.position.y:.2f}, {first_pose.pose.position.z:.2f})')
                self.get_logger().info(f'Last waypoint: ({last_pose.pose.position.x:.2f}, {last_pose.pose.position.y:.2f}, {last_pose.pose.position.z:.2f})')
                self.path_received = True


def main(args=None):
    rclpy.init(args=args)
    test_node = TestPCTPlannerNode()
    
    # 运行一段时间以接收路径消息
    try:
        for i in range(20):  # 等待10秒
            rclpy.spin_once(test_node, timeout_sec=0.5)
            if test_node.path_received:
                test_node.get_logger().info('SUCCESS: Path message received from PCT planner!')
                break
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()