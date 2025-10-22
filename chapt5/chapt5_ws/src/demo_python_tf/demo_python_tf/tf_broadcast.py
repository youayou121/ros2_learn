import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from tf_transformations import quaternion_from_euler
import math
class TFBroadcaster(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.timer_ = self.create_timer(0.1, self.tf_callback)
    def publish_tf(self, frame_id, child_frame_id, transform):
        tf = TransformStamped()
        tf.header.frame_id = frame_id
        tf.child_frame_id = child_frame_id
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform = transform
        self.tf_broadcaster_.sendTransform(tf)
        self.get_logger().info(f'Published static transform from {frame_id} to {child_frame_id}')
    def tf_callback(self):
        frame_id = 'map'
        child_frame_id = 'odom'
        transform = Transform()
        transform.translation.x = 1.0
        transform.translation.y = 1.0
        transform.translation.z = 0.0
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        transform.rotation.x = q[0]
        transform.rotation.y = q[1]
        transform.rotation.z = q[2]
        transform.rotation.w = q[3]
        self.publish_tf(frame_id, child_frame_id, transform)
        

def main():
    rclpy.init()
    node = TFBroadcaster('tf_broadcaster')
    rclpy.spin(node)
    rclpy.shutdown()
