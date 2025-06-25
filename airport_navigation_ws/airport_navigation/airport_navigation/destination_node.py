#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from airport_navigation_interfaces.srv import SetDestination
from airport_navigation_interfaces.msg import CustomWaypoint, GNSSOrigin
from geometry_msgs.msg import PointStamped
from pyproj import Transformer

class DestinationNode(Node):
    def __init__(self):
        super().__init__('destination_node')
        self.get_logger().info('🛰️ Destination Node waiting for GNSS origin...')

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.origin_received = False
        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_alt = 0.0

        self.transformer = None

        self.origin_sub = self.create_subscription(
            GNSSOrigin,
            '/gnss/first_fix',
            self.origin_callback,
            qos
        )

        # Publisher of local ENU coordinates as PointStamped
        self.enu_publisher = self.create_publisher(PointStamped, '/destination_position', qos)
        # Publisher of lat/lon as CustomWaypoint (optional, for reference)
        self.latlon_publisher = self.create_publisher(CustomWaypoint, '/destination_position_latlong', qos)

        self.srv = self.create_service(SetDestination, 'set_destination', self.set_destination_callback)

    def origin_callback(self, msg):
        if not self.origin_received:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            utm_zone = int((self.origin_lon + 180) / 6) + 1
            is_northern = self.origin_lat >= 0
            epsg_code = 32600 + utm_zone if is_northern else 32700 + utm_zone

            self.transformer = Transformer.from_crs("epsg:4326", f"epsg:{epsg_code}", always_xy=True)
            self.origin_x, self.origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)

            self.origin_received = True
            self.get_logger().info(f"✅ GNSS origin received: ({self.origin_lat}, {self.origin_lon}) using EPSG:{epsg_code}")

    def set_destination_callback(self, request, response):
        if not self.origin_received:
            msg = "❌ GNSS origin not yet received"
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        lat = request.latitude
        lon = request.longitude
        alt = request.altitude

        utm_x, utm_y = self.transformer.transform(lon, lat)
        local_x = utm_x - self.origin_x
        local_y = utm_y - self.origin_y

        # Publish as PointStamped for RViz and navigation nodes
        point_msg = PointStamped()
        point_msg.header.frame_id = "map"
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = local_x
        point_msg.point.y = local_y
        point_msg.point.z = alt
        self.enu_publisher.publish(point_msg)

        # Also publish latlon for info/debug (optional)
        latlon_msg = CustomWaypoint()
        latlon_msg.latitude = lat
        latlon_msg.longitude = lon
        latlon_msg.altitude = alt
        latlon_msg.name = "Destination"
        self.latlon_publisher.publish(latlon_msg)

        response.success = True
        response.message = f"📍 Destination set: ENU=({local_x:.2f}, {local_y:.2f}), LatLon=({lat:.6f}, {lon:.6f})"
        self.get_logger().info(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DestinationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
