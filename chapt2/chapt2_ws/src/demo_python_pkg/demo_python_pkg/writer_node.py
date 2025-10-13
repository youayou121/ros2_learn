from demo_python_pkg.person_node import PersonNode
import rclpy
from rclpy.node import Node
class WriterNode(PersonNode):
    def __init__(self, node_name: str, name: str, age: int, book: str) -> None:
        super().__init__(node_name, name, age)
        self.book = book

def main():
    rclpy.init()
    writer = WriterNode("zhangsan_node", "zhangsan", 18, "The Great Gatsby")
    writer.eat("banana")
    print(f"{writer.name}'s book is {writer.book}")
    rclpy.spin(writer)
    rclpy.shutdown()