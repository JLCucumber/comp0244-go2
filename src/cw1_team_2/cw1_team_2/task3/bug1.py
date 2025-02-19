import rclpy
from rclpy.node import Node

class BugPlanner(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} Node Started")

def main(args=None):
    rclpy.init(args=args)
    node = BugPlanner("Bug1")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
