import espeakng
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import threading
import time
class NovelSubNode(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)
        self.get_logger().info(f"Node {node_name} started.")
        self.novel_subscriber_ = self.create_subscription(String, "novel_topic", self.msg_callback, 10)
        self.novels_queue_ = Queue()
        self.speaker = espeakng.Speaker()
        self.speak_thread_ = threading.Thread(target=self.speak_callback)
        self.speak_thread_.start()
    
    def msg_callback(self, msg):
        self.get_logger().info(f"Received novel: {msg.data[:10]}...")
        self.novels_queue_.put(msg.data)
    def speak_callback(self):
        while rclpy.ok():
            if self.novels_queue_.qsize() > 0:
                novel = self.novels_queue_.get()
                self.speaker.say(novel)
                self.get_logger().info(f"Spoken novel: {novel[:10]}...")
                self.speaker.wait()
            else:
                self.get_logger().info("No novels to speak, waiting...")
                time.sleep(1)


def main():
    rclpy.init()
    novel_sub_node = NovelSubNode("novel_sub_node")
    rclpy.spin(novel_sub_node)
    rclpy.shutdown()
    