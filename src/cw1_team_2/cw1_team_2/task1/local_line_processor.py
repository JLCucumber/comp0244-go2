import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class LocalLineProcessor(Node):
    def __init__(self):
        super().__init__('local_line_processor')

        # 订阅 local_map_lines
        self.subscription = self.create_subscription(
            Marker,
            'local_map_lines',
            self.callback_process_lines,
            10)
        self.subscription  # 避免垃圾回收

        # 发布处理后的 local_map_lines_processed
        self.publisher = self.create_publisher(
            Marker,
            'local_map_lines_processed',  # 处理后发布的新 topic
            10)

    def callback_process_lines(self, msg):
        """处理 local_map_lines 并重新发布"""
        self.get_logger().info(f"Received Marker ID: {msg.id}, Points: {len(msg.points)}")

        # 创建新的 Marker 消息
        processed_marker = Marker()
        processed_marker.header = msg.header
        processed_marker.type = msg.type
        processed_marker.action = msg.action
        processed_marker.id = msg.id
        processed_marker.scale = msg.scale
        processed_marker.color = msg.color

        # 🔹 数据预处理（例如：滤波、平滑、转换等）
        processed_marker.points = self.process_lines(msg.points)

        # 发布新的 Marker
        self.publisher.publish(processed_marker)
        self.get_logger().info(f"Published processed Marker ID: {msg.id}")

    def process_lines(self, points):
        """示例：对线段点云数据进行简单滤波"""
        processed_points = []
        for point in points:
            # 这里可以应用滤波、平滑、降采样等
            if point.x**2 + point.y**2 > 0.01:  # 过滤掉太小的点
                processed_points.append(point)
        return processed_points

def main(args=None):
    rclpy.init(args=args)
    node = LocalLineProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
