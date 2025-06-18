#route_manager_node.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from airport_navigation_interfaces.srv import SetRoute
from airport_navigation_interfaces.msg import CustomWaypoint
from std_msgs.msg import Bool
from geometry_msgs.msg import PointStamped
import pyproj

class RouteManagerNode(Node):
    def __init__(self):
        super().__init__('route_manager_node')
        self.declare_parameter('utm_zone', 14)
        self.utm_zone = self.get_parameter('utm_zone').get_parameter_value().integer_value

        self.route = []
        self.current_index = 0

        # Standard QoS
        self.current_pub = self.create_publisher(PointStamped, 'current_target_position', 10)
        self.debug_pub = self.create_publisher(PointStamped, 'debug/current_waypoint', 10)

        # TRANSIENT_LOCAL QoS for waypoint debug so ros2 topic echo always works
        qos_transient = QoSProfile(depth=10)
        qos_transient.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.latlon_pub = self.create_publisher(CustomWaypoint, 'all_waypoints_latlon', qos_transient)

        self.srv = self.create_service(SetRoute, 'set_route', self.handle_set_route)
        self.sub = self.create_subscription(Bool, 'waypoint_reached', self.handle_waypoint_reached, 10)

        self.utm_proj = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')

    def handle_set_route(self, request, response):
        self.route = []
        self.get_logger().info(f"📦 Received {len(request.waypoints)} waypoints")

        try:
            for idx, wp in enumerate(request.waypoints):
                lat = wp.latitude
                lon = wp.longitude
                alt = wp.altitude
                name = wp.name

                # Convert lat/lon to UTM
                utm_x, utm_y = self.utm_proj(lon, lat)  # lon, lat order for pyproj

                # UTM in PointStamped
                msg = PointStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x = utm_x
                msg.point.y = utm_y
                msg.point.z = alt

                self.route.append(msg)

                # Publish lat/lon with name
                latlon_msg = CustomWaypoint()
                latlon_msg.latitude = lat
                latlon_msg.longitude = lon
                latlon_msg.altitude = alt
                latlon_msg.name = name
                self.latlon_pub.publish(latlon_msg)

                self.get_logger().info(f"✅ Waypoint {idx+1}: '{name}' lat={lat}, lon={lon}, UTM=({utm_x}, {utm_y})")

            self.current_index = 0
            if self.route:
                self.publish_current_target()

            response.success = True
            response.message = f"✅ Route received with {len(self.route)} waypoints."

        except Exception as e:
            response.success = False
            response.message = f"❌ Failed to parse waypoints: {e}"
            self.get_logger().error(response.message)

        return response

    def publish_current_target(self):
        if self.current_index < len(self.route):
            msg = self.route[self.current_index]
            self.current_pub.publish(msg)
            self.debug_pub.publish(msg)
            self.get_logger().info(f"📍 Published waypoint {self.current_index + 1}/{len(self.route)}")
        else:
            self.get_logger().info("✅ All waypoints completed.")

    def handle_waypoint_reached(self, msg):
        if msg.data and self.current_index < len(self.route) - 1:
            self.current_index += 1
            self.publish_current_target()

def main(args=None):
    rclpy.init(args=args)
    node = RouteManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
