import rclpy
from status_interfaces.msg import SystemStatus
from rclpy.node import Node
import psutil
import platform

class SystemStatusPub(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.status_publisher = self.create_publisher(SystemStatus, "/sys_status", 10)
        self.timer_ = self.create_timer(1.0, self.timer_callback)
    def timer_callback(self):
        cpu_percent = psutil.cpu_percent()
        virtual_memory = psutil.virtual_memory()
        net_io_counters = psutil.net_io_counters()
        msg = SystemStatus()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.host_name = platform.node()
        msg.cpu_percent = cpu_percent
        msg.memory_percent = virtual_memory.percent
        msg.memory_total = virtual_memory.total / 1024 / 1024
        msg.memory_available = virtual_memory.available / 1024 / 1024
        msg.net_sent = net_io_counters.bytes_sent / 1024 / 1024
        msg.net_recv = net_io_counters.bytes_recv / 1024 / 1024
        self.get_logger().info(f"publish {str(msg)}")
        self.status_publisher.publish(msg)

def main():
    rclpy.init()
    sys_status_pub = SystemStatusPub("sys_status_pub_node")
    rclpy.spin(sys_status_pub)
    rclpy.shutdown()
