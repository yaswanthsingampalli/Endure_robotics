#!/usr/bin/env python3
#gnss_source_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from pyproj import CRS, Transformer
from rclpy.qos import QoSProfile, ReliabilityPolicy

class GNSSPositionNode(Node):
    def __init__(self):
        super().__init__('gnss_position_node')

        # Declare parameters
        self.declare_parameter('origin_lat', 37.6188056)
        self.declare_parameter('origin_lon', -122.3754167)
        self.declare_parameter('origin_alt', 4.0)
        self.declare_parameter('utm_zone', 10)

        # Get parameters and cast types
        self.origin_lat = float(self.get_parameter('origin_lat').value)
        self.origin_lon = float(self.get_parameter('origin_lon').value)
        self.origin_alt = float(self.get_parameter('origin_alt').value)
        utm_zone = int(self.get_parameter('utm_zone').value)

        # Setup CRS and Transformer
        self.wgs84 = CRS.from_epsg(4326)
        utm_crs_code = 32600 + utm_zone  # EPSG for UTM zones in northern hemisphere
        self.utm_proj = CRS.from_epsg(utm_crs_code)
        self.transformer = Transformer.from_crs(self.wgs84, self.utm_proj, always_xy=True)

        # Origin UTM coordinates
        self.origin_x, self.origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)

        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.publisher_ = self.create_publisher(Point, 'source_position', 10)
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gnss_callback,
            qos_profile
        )

        self.get_logger().info(f"GNSS node initialized with origin at lat={self.origin_lat}, lon={self.origin_lon}")

    def gnss_callback(self, msg: NavSatFix):
        try:
            if msg.status.status < 0:
                self.get_logger().warn('No GPS fix available', throttle_duration_sec=10.0)
                return

            # Convert lat/lon to UTM coordinates
            x, y = self.transformer.transform(msg.longitude, msg.latitude)

            # Convert to local ENU frame
            local_x = x - self.origin_x
            local_y = y - self.origin_y
            local_z = msg.altitude - self.origin_alt

            point_msg = Point(x=local_x, y=local_y, z=local_z)
            self.publisher_.publish(point_msg)

            self.get_logger().debug(
                f"Position: E={local_x:.2f}m, N={local_y:.2f}m, U={local_z:.2f}m",
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f"GNSS processing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = GNSSPositionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

