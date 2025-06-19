#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from airport_navigation_interfaces.srv import SetRoute
from airport_navigation_interfaces.msg import CustomWaypoint
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
from pyproj import Transformer
from threading import Lock

class RouteManagerNode(Node):
    def __init__(self):
        super().__init__('route_manager_node')

        # Declare origin parameters
        self.origin_lat = self.declare_parameter('origin_latitude', 0.0).get_parameter_value().double_value
        self.origin_lon = self.declare_parameter('origin_longitude', 0.0).get_parameter_value().double_value

        # Determine UTM zone and EPSG code
        utm_zone = int((self.origin_lon + 180) / 6) + 1
        is_northern = self.origin_lat >= 0
        epsg_code = 32600 + utm_zone if is_northern else 32700 + utm_zone
        self.get_logger().info(f'Using UTM zone {utm_zone} EPSG:{epsg_code} for coordinate transformation')

        # Set up transformer
        self.transformer = Transformer.from_crs("epsg:4326", f"epsg:{epsg_code}", always_xy=True)
        self.origin_x, self.origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)

        # Internal waypoint lists
        self.route_latlon = []  # List of CustomWaypoint (lat/lon)
        self.route_enu = []     # List of CustomWaypoint (ENU)

        self.current_index = 0
        self.lock = Lock()

        # Publishers
        self.current_pub = self.create_publisher(PointStamped, 'current_target_position', 10)
        self.debug_pub = self.create_publisher(PointStamped, 'debug/current_waypoint', 10)
        self.latlon_pub = self.create_publisher(CustomWaypoint, 'route_manager_node/debug/waypoints_latlon', 10)
        self.enu_pub = self.create_publisher(CustomWaypoint, 'route_manager_node/debug/waypoints_enu', 10)

        # Service and subscription
        self.srv = self.create_service(SetRoute, 'set_route', self.handle_set_route)
        self.sub = self.create_subscription(Bool, 'route_manager_node/waypoint_reached', self.handle_waypoint_reached, 10)

        # Timers
        self.latlon_timer = self.create_timer(1.0, self.publish_all_latlon)
        self.enu_timer = self.create_timer(1.0, self.publish_all_enu)

    def handle_set_route(self, request, response):
        with self.lock:
            self.route_latlon.clear()
            self.route_enu.clear()
            self.get_logger().info(f"Received {len(request.waypoints)} waypoints")

            try:
                for idx, wp in enumerate(request.waypoints):
                    lat = wp.latitude
                    lon = wp.longitude
                    alt = wp.altitude
                    name = wp.name or f"Waypoint {idx + 1}"

                    # Convert to UTM and compute ENU
                    utm_x, utm_y = self.transformer.transform(lon, lat)
                    local_x = utm_x - self.origin_x  # Easting
                    local_y = utm_y - self.origin_y  # Northing

                    # Lat/Lon waypoint
                    latlon_msg = CustomWaypoint()
                    latlon_msg.latitude = lat
                    latlon_msg.longitude = lon
                    latlon_msg.altitude = alt
                    latlon_msg.name = name
                    self.route_latlon.append(latlon_msg)

                    # ENU waypoint (remap: Y => latitude, X => longitude for display)
                    enu_msg = CustomWaypoint()
                    enu_msg.latitude = local_y     # Northing
                    enu_msg.longitude = local_x    # Easting
                    enu_msg.altitude = alt
                    enu_msg.name = name
                    self.route_enu.append(enu_msg)

                    self.get_logger().info(
                        f"✅ Waypoint {idx+1}: '{name}' lat={lat}, lon={lon}, "
                        f"ENU=({local_x:.2f}, {local_y:.2f})"
                    )

                self.current_index = 0
                if self.route_enu:
                    self.publish_current_target()

                response.success = True
                response.message = f"Route received with {len(self.route_enu)} waypoints."

            except Exception as e:
                self.get_logger().error(f"Failed to process route: {e}")
                response.success = False
                response.message = str(e)

        return response

    def publish_current_target(self):
        if self.current_index < len(self.route_enu):
            point_msg = PointStamped()
            point_msg.header.frame_id = "map"
            point_msg.header.stamp = self.get_clock().now().to_msg()
            enu = self.route_enu[self.current_index]
            point_msg.point.x = enu.longitude  # ENU X (Easting)
            point_msg.point.y = enu.latitude   # ENU Y (Northing)
            point_msg.point.z = enu.altitude

            self.current_pub.publish(point_msg)
            self.debug_pub.publish(point_msg)
            self.get_logger().info(f"📍 Published current target: {enu.name}")
        else:
            self.get_logger().info("✅ All waypoints have been reached.")

    def handle_waypoint_reached(self, msg):
        if msg.data:
            if self.current_index < len(self.route_enu) - 1:
                self.current_index += 1
                self.publish_current_target()
            else:
                self.get_logger().info("🏁 Final waypoint reached.")

    def publish_all_latlon(self):
        with self.lock:
            for wp in self.route_latlon:
                self.latlon_pub.publish(wp)

    def publish_all_enu(self):
        with self.lock:
            for wp in self.route_enu:
                self.enu_pub.publish(wp)

def main(args=None):
    rclpy.init(args=args)
    node = RouteManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
