#!/usr/bin/env python3
# navigation_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, ReliabilityPolicy
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.declare_parameter('distance_threshold', 1.0)
        self.declare_parameter('logging_frequency', 1.0)

        self.source = None
        self.destination = None
        self.last_log_time = self.get_clock().now()

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Point, 'source_position', self.source_callback, qos_profile)
        self.create_subscription(Point, 'destination_position', self.destination_callback, qos_profile)

        self.get_logger().info("Navigation node ready")

    def source_callback(self, msg):
        self.source = msg
        self.try_navigate()

    def destination_callback(self, msg):
        self.destination = msg
        self.try_navigate()

    def try_navigate(self):
        if self.source is None or self.destination is None:
            return

        current_time = self.get_clock().now()
        time_since_last_log = (current_time - self.last_log_time).nanoseconds * 1e-9

        logging_freq = float(self.get_parameter('logging_frequency').value)
        if time_since_last_log < logging_freq:
            return

        dx = self.destination.x - self.source.x
        dy = self.destination.y - self.source.y
        distance = math.sqrt(dx**2 + dy**2)

        threshold = float(self.get_parameter('distance_threshold').value)
        status = "Reached" if distance <= threshold else "Navigating"

        bearing_deg = math.degrees(math.atan2(dy, dx))
        self.get_logger().info(
            f"{status}: Distance={distance:.2f}m | Bearing={bearing_deg:.1f}° | "
            f"Current: ({self.source.x:.2f}, {self.source.y:.2f}) | "
            f"Target: ({self.destination.x:.2f}, {self.destination.y:.2f})"
        )

        self.last_log_time = current_time

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

