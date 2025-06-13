# rviz_debug_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RvizDebugNode(Node):
    def __init__(self):
        super().__init__('rviz_debug_node')
        self.get_logger().info('[RVIZ DEBUG] Starting debug topic monitor...')
        
        # Subscribe to all expected debug topics
        self.debug_topics = [
            'debug/gnss_position',
            'debug/route_status',
            'debug/waypoint_info',
            'debug/path_following',
            'debug/system_diagnostics',
        ]
        self.subscribers = []
        for topic in self.debug_topics:
            sub = self.create_subscription(
                String,
                topic,
                self.make_callback(topic),
                10
            )
            self.subscribers.append(sub)

    def make_callback(self, topic_name):
        def callback(msg):
            self.get_logger().info(f"[RVIZ DEBUG] {topic_name}: {msg.data}")
        return callback

def main(args=None):
    rclpy.init(args=args)
    node = RvizDebugNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
