import rclpy
from rclpy.node import Node
class PersonNode(Node):
    def __init__(self, node_name: str, name: str, age: int) -> None:
        super().__init__(node_name)
        self.name = name
        self.age = age
        pass
    def eat(self, food : str) -> None:
        self.get_logger().info(f"{self.name}, {self.age} years old. likes eating {food}")


def main():
    rclpy.init()
    zhangsan = PersonNode("zhangsan_node", "zhangsan", 18)
    zhangsan.eat("apple")
    rclpy.spin(zhangsan)
    rclpy.shutdown()