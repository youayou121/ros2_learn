import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue
class NovelPubNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.get_logger().info(f"Node {node_name} started.")
        self.novels_queue_ = Queue()
        self.novel_publisher_ = self.create_publisher(String, "novel_topic", 10)
        self.create_timer(1.0, self.timer_callback)
    def timer_callback(self):
        if self.novels_queue_.qsize() > 0:
            novel = self.novels_queue_.get()
            msg = String()
            msg.data = novel
            self.novel_publisher_.publish(msg)
            self.get_logger().info(f"Published novel: {novel[:10]}...")

    def download(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        text = response.text
        self.get_logger().info(f"Downloaded content from {url}:\n{text[:10]}...")
        for line in text.splitlines():
            self.novels_queue_.put(line)
        
        
def main():
    rclpy.init()
    novel_pub_node = NovelPubNode("novel_pub_node")
    novel_pub_node.download('http://0.0.0.0:8000/novels1.txt')
    rclpy.spin(novel_pub_node)
    rclpy.shutdown()
