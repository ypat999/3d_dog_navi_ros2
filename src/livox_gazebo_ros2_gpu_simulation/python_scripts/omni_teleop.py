#!/usr/bin/env python3
"""ROS 2 全向移动键盘控制脚本（适用于平面移动小车）"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select  # 新增：用于非阻塞读取

# 键位映射：键名 → (linear.x, linear.y, angular.z)
KEY_MAPPING = {
    'w': (3.0, 0.0, 0.0),    # 前进
    's': (-3.0, 0.0, 0.0),   # 后退
    'a': (0.0, 3.0, 0.0),    # 向左横向移动
    'd': (0.0, -3.0, 0.0),   # 向右横向移动
    'q': (0.0, 0.0, 5.0),    # 向左旋转
    'e': (0.0, 0.0, -5.0),   # 向右旋转
    ' ': (0.0, 0.0, 0.0)     # 空格：停止所有运动
}

class OmniTeleopNode(Node):
    def __init__(self):
        super().__init__("omni_teleop_node")  
        self.vel_publisher = self.create_publisher(
            Twist, 
            "/cmd_vel",  
            10  
        )
        
        # 打印操作提示
        self.get_logger().info("=" * 50)
        self.get_logger().info("全向移动键盘控制已启动！")
        self.get_logger().info("操作说明：")
        self.get_logger().info("w: 前进    s: 后退    a: 向左横移    d: 向右横移")
        self.get_logger().info("q: 左转    e: 右转    空格: 清除状态")
        self.get_logger().info("ESC: 退出程序")
        self.get_logger().info("=" * 50)

    def get_keyboard_input(self):
        """非阻塞读取键盘输入（超时0.1秒，无输入返回None）"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            # 检查是否有输入（超时0.1秒）
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:  # 有输入时读取
                key = sys.stdin.read(1)
            else:  # 超时无输入
                key = None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """主循环：持续读取键盘输入并发布速度指令"""
        vel_msg = Twist()
        while rclpy.ok():
            key = self.get_keyboard_input()
            
            # 处理退出按键（ESC键）
            if key == '\x1b':
                self.get_logger().info("退出控制程序，停止小车运动")
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.angular.z = 0.0
                self.vel_publisher.publish(vel_msg)
                break

            # 处理空格键（停止所有运动）
            if key == ' ':
                self.get_logger().info("传入空格，停止小车运动")
                vel_msg.linear.x = 0.0
                vel_msg.linear.y = 0.0
                vel_msg.angular.z = 0.0
                self.vel_publisher.publish(vel_msg)
            
            # 有有效按键时（排除空格键），发布对应速度
            elif key in KEY_MAPPING and key != ' ':
                vel_msg.linear.x += KEY_MAPPING[key][0] * 0.01
                vel_msg.linear.y += KEY_MAPPING[key][1] * 0.01
                vel_msg.angular.z += KEY_MAPPING[key][2] * 0.03
                self.vel_publisher.publish(vel_msg)
            
            # 无按键输入（超时）或无效按键时，发布停止指令
            else:
                vel_msg.linear.x *= 0.9
                vel_msg.linear.y *= 0.9
                vel_msg.angular.z *= 0.99
                self.vel_publisher.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OmniTeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        vel_msg = Twist()
        node.vel_publisher.publish(vel_msg)
        node.get_logger().info("捕获 Ctrl+C，停止小车运动")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
