#!/usr/bin/env python3
# destination_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from airport_navigation_interfaces.srv import SetDestination
from airport_navigation_interfaces.msg import CustomWaypoint
from pyproj import Transformer

class DestinationNode(Node):
    def __init__(self):
        super().__init__('destination_node')

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Publish ENU using CustomWaypoint with proper field labels
        self.enu_publisher = self.create_publisher(CustomWaypoint, 'destination_node/destination_position', qos)

        # Publish raw lat/lon as CustomWaypoint
        self.latlon_publisher = self.create_publisher(CustomWaypoint, 'destination_node/coordinate_position_latLong/destination_position_latlong', qos)

        self.srv = self.create_service(SetDestination, 'set_destination', self.set_destination_callback)
        self.get_logger().info('✅ Destination Node is up and running.')

        # Load origin
        self.origin_lat = self.declare_parameter('origin_latitude', 0.0).get_parameter_value().double_value
        self.origin_lon = self.declare_parameter('origin_longitude', 0.0).get_parameter_value().double_value

        # Coordinate transformation setup
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

            # Convert to local ENU
            utm_x, utm_y = self.transformer.transform(lon, lat)
            local_x = utm_x - self.origin_x
            local_y = utm_y - self.origin_y

            # ENU as CustomWaypoint (remap ENU to lat/long labels for debug clarity)
            enu_msg = CustomWaypoint()
            enu_msg.latitude = local_y     # northing → shown as "latitude"
            enu_msg.longitude = local_x    # easting  → shown as "longitude"
            enu_msg.altitude = alt
            enu_msg.name = "Destination"
            self.enu_publisher.publish(enu_msg)

            # LatLon as CustomWaypoint
            latlon_msg = CustomWaypoint()
            latlon_msg.latitude = lat
            latlon_msg.longitude = lon
            latlon_msg.altitude = alt
            latlon_msg.name = "Destination"
            self.latlon_publisher.publish(latlon_msg)

            response.success = True
            response.message = f"📍 Destination set:\n - ENU=({local_x:.2f}, {local_y:.2f})\n - LatLon=({lat:.6f}, {lon:.6f})"
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
