import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__("minimal_node")
        self.get_logger().info("Hello from ROS2 Node!")

def main(args=None):
    rclpy.init(args=args)
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()