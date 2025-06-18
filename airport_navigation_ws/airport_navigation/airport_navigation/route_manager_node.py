import rclpy
from rclpy.node import Node
from airport_navigation_interfaces.srv import SetRoute
from airport_navigation_interfaces.msg import CustomWaypoint
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, PointStamped
from pyproj import Transformer
from threading import Lock

class RouteManagerNode(Node):
    def __init__(self):
        super().__init__('route_manager_node')

        # Declare and get origin params for ENU transform
        self.origin_lat = self.declare_parameter('origin_latitude', 0.0).get_parameter_value().double_value
        self.origin_lon = self.declare_parameter('origin_longitude', 0.0).get_parameter_value().double_value

        # Calculate UTM zone and EPSG code
        utm_zone = int((self.origin_lon + 180) / 6) + 1
        is_northern = self.origin_lat >= 0
        epsg_code = 32600 + utm_zone if is_northern else 32700 + utm_zone
        self.get_logger().info(f'Using UTM zone {utm_zone} EPSG:{epsg_code} for coordinate transformation')

        # Set up transformer to convert lat/lon to UTM
        self.transformer = Transformer.from_crs("epsg:4326", f"epsg:{epsg_code}", always_xy=True)
        self.origin_x, self.origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)

        # Internal storage for waypoints
        self.route_enu = []       # List of geometry_msgs/Point for ENU waypoints
        self.route_latlon = []    # List of CustomWaypoint for lat/lon waypoints
        self.current_index = 0

        # Publishers
        self.current_pub = self.create_publisher(PointStamped, 'current_target_position', 10)
        self.debug_pub = self.create_publisher(PointStamped, 'debug/current_waypoint', 10)
        self.latlon_pub = self.create_publisher(CustomWaypoint, 'debug/waypoints_latlon', 10)
        self.enu_pub = self.create_publisher(Point, 'debug/waypoints_enu', 10)

        # Service and subscription
        self.srv = self.create_service(SetRoute, 'set_route', self.handle_set_route)
        self.sub = self.create_subscription(Bool, 'waypoint_reached', self.handle_waypoint_reached, 10)

        # Lock for thread safety
        self.lock = Lock()

        # Timers to repeatedly publish waypoints at 1 Hz
        self.timer_latlon = self.create_timer(1.0, self.publish_all_latlon)
        self.timer_enu = self.create_timer(1.0, self.publish_all_enu)

    def handle_set_route(self, request, response):
        with self.lock:
            self.route_enu.clear()
            self.route_latlon.clear()

            self.get_logger().info(f"Received {len(request.waypoints)} waypoints")

            try:
                for idx, wp in enumerate(request.waypoints):
                    lat = wp.latitude
                    lon = wp.longitude
                    alt = wp.altitude
                    name = wp.name

                    # Convert lat/lon to UTM (ENU relative to origin)
                    utm_x, utm_y = self.transformer.transform(lon, lat)
                    local_x = utm_x - self.origin_x
                    local_y = utm_y - self.origin_y

                    # ENU waypoint as Point message
                    enu_point = Point()
                    enu_point.x = local_x
                    enu_point.y = local_y
                    enu_point.z = alt
                    self.route_enu.append(enu_point)

                    # Lat/Lon waypoint as CustomWaypoint message
                    latlon_msg = CustomWaypoint()
                    latlon_msg.latitude = lat
                    latlon_msg.longitude = lon
                    latlon_msg.altitude = alt
                    latlon_msg.name = name
                    self.route_latlon.append(latlon_msg)

                    self.get_logger().info(f"✅ Waypoint {idx+1}: '{name}' lat={lat}, lon={lon}, ENU=({local_x:.2f}, {local_y:.2f})")

                self.current_index = 0
                if self.route_enu:
                    self.publish_current_target()

                response.success = True
                response.message = f"✅ Route received with {len(self.route_enu)} waypoints."

            except Exception as e:
                response.success = False
                response.message = f"❌ Failed to parse waypoints: {e}"
                self.get_logger().error(response.message)

        return response

    def publish_current_target(self):
        if self.current_index < len(self.route_enu):
            point = self.route_enu[self.current_index]
            msg = PointStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point = point

            self.current_pub.publish(msg)
            self.debug_pub.publish(msg)
            self.get_logger().info(f"📍 Published waypoint {self.current_index + 1}/{len(self.route_enu)}")
        else:
            self.get_logger().info("✅ All waypoints completed.")

    def handle_waypoint_reached(self, msg):
        if msg.data:
            if self.current_index < len(self.route_enu) - 1:
                self.current_index += 1
                self.publish_current_target()

    def publish_all_latlon(self):
        with self.lock:
            for wp in self.route_latlon:
                self.latlon_pub.publish(wp)

    def publish_all_enu(self):
        with self.lock:
            for pt in self.route_enu:
                self.enu_pub.publish(pt)

def main(args=None):
    rclpy.init(args=args)
    node = RouteManagerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
