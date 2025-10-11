import rclpy
from rclpy.node import Node
def main():
    rclpy.init()
    node = Node("ros2_python_node")
    node.get_logger().info("Hello ROS2")
    node.get_logger().warn("Hello ROS2")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()