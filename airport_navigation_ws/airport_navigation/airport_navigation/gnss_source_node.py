#!/usr/bin/env python3
#gnss_source_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from pyproj import CRS, Transformer
from rclpy.qos import QoSProfile, ReliabilityPolicy

class GNSSSourceNode(Node):
    def __init__(self):
        super().__init__('gnss_source_node')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.subscription = self.create_subscription(NavSatFix, '/fix', self.gnss_callback, qos)
        self.publisher_ = self.create_publisher(Point, 'current_position', 10)
        self.debug_pub = self.create_publisher(Point, 'debug/gnss_local_position', 10)

        self.first_fix = None
        self.transformer = None

        self.get_logger().info('[GNSS] Node started. Waiting for first GPS fix...')

    def gnss_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warn('[GNSS] No GPS fix available')
            return

        if self.first_fix is None:
            self.first_fix = (msg.longitude, msg.latitude)
            utm_zone = int((msg.longitude + 180) / 6) + 1
            self.wgs84 = CRS.from_epsg(4326)
            self.utm = CRS.from_epsg(32600 + utm_zone)
            self.transformer = Transformer.from_crs(self.wgs84, self.utm, always_xy=True)
            self.origin_x, self.origin_y = self.transformer.transform(msg.longitude, msg.latitude)
            self.origin_alt = msg.altitude
            self.get_logger().info(
                f'[GNSS] Initialized ENU origin at lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}, UTM zone={utm_zone}'
            )

        x, y = self.transformer.transform(msg.longitude, msg.latitude)
        local_x = x - self.origin_x
        local_y = y - self.origin_y
        local_z = msg.altitude - self.origin_alt

        pt = Point(x=local_x, y=local_y, z=local_z)
        self.publisher_.publish(pt)
        self.debug_pub.publish(pt)

        self.get_logger().debug(f'[GNSS] Local position: x={local_x:.2f}, y={local_y:.2f}, z={local_z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = GNSSSourceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
