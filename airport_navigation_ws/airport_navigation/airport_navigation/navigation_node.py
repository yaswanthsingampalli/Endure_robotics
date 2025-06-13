#navigation_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.declare_parameter('distance_threshold', 1.0)
        self.declare_parameter('logging_frequency', 1.0)
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.logging_frequency = self.get_parameter('logging_frequency').value

        self.source = None
        self.destination = None
        self.last_log_time = self.get_clock().now()

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.subscription_source = self.create_subscription(Point, 'source_position', self.source_callback, qos_profile)
        self.subscription_destination = self.create_subscription(Point, 'destination_position', self.destination_callback, qos_profile)
        self.debug_pub = self.create_publisher(Point, 'debug/source_position', 10)
        self.navigation_complete_publisher = self.create_publisher(Bool, 'navigation_complete', 10)

        self.get_logger().info("[NAV] Navigation Node started.")

    def source_callback(self, msg):
        self.source = msg
        self.debug_pub.publish(msg)
        self.try_navigate()

    def destination_callback(self, msg):
        self.destination = msg
        self.try_navigate()

    def try_navigate(self):
        if self.source is None or self.destination is None:
            return

        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds * 1e-9 < self.logging_frequency:
            return

        dx = self.destination.x - self.source.x
        dy = self.destination.y - self.source.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance <= self.distance_threshold:
            status = "[NAV] Reached Destination"
            self.navigation_complete_publisher.publish(Bool(data=True))
        else:
            status = "[NAV] Navigating"

        bearing_deg = math.degrees(math.atan2(dy, dx))
        self.get_logger().info(
            f"{status}: Distance={distance:.2f}m | Bearing={bearing_deg:.1f}° | "
            f"Source=({self.source.x:.2f}, {self.source.y:.2f}) → Target=({self.destination.x:.2f}, {self.destination.y:.2f})"
        )
        self.last_log_time = now

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
