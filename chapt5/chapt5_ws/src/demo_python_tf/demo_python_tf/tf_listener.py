import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
class TFListener(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.buffer_, self)
        self.timer_ = self.create_timer(1, self.get_tf)
    def get_tf(self):
        try:
            target_frame = 'laser_link'
            source_frame = 'odom'
            result = self.buffer_.lookup_transform(target_frame, source_frame, rclpy.time.Time(seconds=0.0), rclpy.time.Duration(seconds=1.0))
            transform = result.transform
            self.get_logger().info(f'Get tf {source_frame} in {target_frame}: {transform.translation}')
            self.get_logger().info(f'Get rot: {transform.rotation}')
            rpy = euler_from_quaternion([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
            self.get_logger().info(f'Get rot(RPY): {rpy}')
        except Exception as e:
            self.get_logger().warn(f'Could not get tf, reason: {e}')
            


def main():
    rclpy.init()
    node = TFListener('tf_listener')
    rclpy.spin(node)
    rclpy.shutdown()
