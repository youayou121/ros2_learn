import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from tf_transformations import quaternion_from_euler
import math
class StaticTFBroadcaster(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.static_broadcaster_laser_in_base = StaticTransformBroadcaster(self)
        self.static_broadcaster_base_in_odom = StaticTransformBroadcaster(self)
    def publish_static_tf(self, broadcaster, frame_id, child_frame_id, transform):
        tf = TransformStamped()
        tf.header.frame_id = frame_id
        tf.child_frame_id = child_frame_id
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.transform = transform
        broadcaster.sendTransform(tf)
        self.get_logger().info(f'Published static transform from {frame_id} to {child_frame_id}')
def main():
    rclpy.init()
    node = StaticTFBroadcaster('static_tf_broadcaster')
    frame_id = 'base_link'
    child_frame_id = 'laser_link'
    transform = Transform()
    transform.translation.x = 1.0
    transform.translation.y = 0.0
    transform.translation.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, 0)
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]
    node.publish_static_tf(node.static_broadcaster_laser_in_base, frame_id, child_frame_id, transform)
    frame_id = 'odom'
    child_frame_id = 'base_link'
    transform = Transform()
    transform.translation.x = 0.0
    transform.translation.y = 0.0
    transform.translation.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, 0.0)
    transform.rotation.x = q[0]
    transform.rotation.y = q[1]
    transform.rotation.z = q[2]
    transform.rotation.w = q[3]
    node.publish_static_tf(node.static_broadcaster_base_in_odom, frame_id, child_frame_id, transform)
    rclpy.spin(node)
    rclpy.shutdown()
