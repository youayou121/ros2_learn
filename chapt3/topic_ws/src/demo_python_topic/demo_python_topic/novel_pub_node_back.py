import rclpy
from rclpy.node import Node

class NovelPubNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.get_logger().info(f"Node {node_name} started.")

def main():
    rclpy.init()
    novel_pub_node = NovelPubNode("novel_pub_node")
    rclpy.spin(novel_pub_node)
    rclpy.shutdown()
    