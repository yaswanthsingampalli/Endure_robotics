#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from airport_navigation_interfaces.srv import SetDestination
from pyproj import Transformer

class DestinationNode(Node):
    def __init__(self):
        super().__init__('destination_node')

        # Durable QoS so echoing works even after initial publishing
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.publisher = self.create_publisher(Point, 'destination_position', qos)
        self.debug_publisher = self.create_publisher(Point, 'coordinate_position_latLong/destination_position_latlong', qos)

        self.srv = self.create_service(SetDestination, 'set_destination', self.set_destination_callback)
        self.get_logger().info('✅ Destination Node is up and running.')

        # Origin declared from launch config
        self.origin_lat = self.declare_parameter('origin_latitude', 0.0).get_parameter_value().double_value
        self.origin_lon = self.declare_parameter('origin_longitude', 0.0).get_parameter_value().double_value

        utm_zone = int((self.origin_lon + 180) / 6) + 1
        is_northern = self.origin_lat >= 0
        epsg_code = 32600 + utm_zone if is_northern else 32700 + utm_zone
        self.get_logger().info(f'Using UTM zone {utm_zone} EPSG:{epsg_code} for coordinate transformation')

        self.transformer = Transformer.from_crs("epsg:4326", f"epsg:{epsg_code}", always_xy=True)
        self.origin_x, self.origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)

    def set_destination_callback(self, request, response):
        try:
            lat = request.latitude
            lon = request.longitude
            alt = request.altitude

            x, y = self.transformer.transform(lon, lat)
            local_x = x - self.origin_x
            local_y = y - self.origin_y

            point = Point()
            point.x = local_x
            point.y = local_y
            point.z = alt

            self.publisher.publish(point)
            self.debug_publisher.publish(point)

            response.success = True
            response.message = f"Destination set to ENU ({point.x:.2f}, {point.y:.2f})"
            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = f"❌ Failed to set destination: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = DestinationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
