import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile
import copy

class OdomTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        
        # Initialize the TransformBroadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a subscription to the /odom topic
        qos_profile = QoSProfile(depth=10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile)
        
        self.get_logger().info('Odom to TF Broadcaster started')
    
    def odom_callback(self, msg: Odometry):
        # Create a TransformStamped message
        t_odom = TransformStamped()
        
        # Set the timestamp to the time of the received message
        t_odom.header.stamp = msg.header.stamp
        
        # Set the frame IDs
        t_odom.header.frame_id = "odom"
        t_odom.child_frame_id = "base_link"
        
        # Set the translation
        t_odom.transform.translation.x = msg.pose.pose.position.x
        t_odom.transform.translation.y = msg.pose.pose.position.y
        t_odom.transform.translation.z = msg.pose.pose.position.z
        
        # Set the rotation
        t_odom.transform.rotation = msg.pose.pose.orientation

        # Base footprint
        t_base_footprint = copy.deepcopy(t_odom)
        t_base_footprint.header.frame_id = "base_link" 
        t_base_footprint.child_frame_id = "base_footprint"
        t_base_footprint.transform.translation.z = 0.0 

        # Broadcast the transforms
        # odom -> base_link based on ground truth plugin
        # base_link -> base_footprint projecting to z=0
        self.tf_broadcaster.sendTransform(t_base_footprint)
        self.tf_broadcaster.sendTransform(t_odom)

def main(args=None):
    rclpy.init(args=args)
    
    node = OdomTransformBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()