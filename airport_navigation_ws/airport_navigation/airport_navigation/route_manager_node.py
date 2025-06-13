#route_manager_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from airport_navigation_interfaces.srv import SetRoute
from pyproj import CRS, Transformer
import requests

class RouteManagerNode(Node):
    def __init__(self):
        super().__init__('route_manager_node')
        self.declare_parameter('origin_lat', 0.0)
        self.declare_parameter('origin_lon', 0.0)
        self.declare_parameter('utm_zone', 10)

        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.utm_zone = self.get_parameter('utm_zone').value

        self.wgs84 = CRS.from_epsg(4326)
        utm_crs_code = 32600 + int(self.utm_zone)
        self.utm_proj = CRS.from_epsg(utm_crs_code)
        self.transformer = Transformer.from_crs(self.wgs84, self.utm_proj, always_xy=True)

        self.origin_x, self.origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)

        self.waypoints = []
        self.current_index = 0

        self.current_target_pub = self.create_publisher(Point, 'current_target_position', 10)
        self.debug_pub = self.create_publisher(Point, 'debug/current_waypoint', 10)
        self.route_set_srv = self.create_service(SetRoute, 'set_route', self.set_route_callback)
        self.waypoint_reached_sub = self.create_subscription(Bool, 'waypoint_reached', self.waypoint_reached_callback, 10)

        self.get_logger().info('[ROUTE] Route Manager Node initialized.')

    def geocode_place(self, place_name):
        url = "https://nominatim.openstreetmap.org/search"
        params = {'q': place_name, 'format': 'json', 'limit': 1}
        try:
            response = requests.get(url, params=params, headers={'User-Agent': 'ros2-robot'})
            data = response.json()
            if data:
                lat = float(data[0]['lat'])
                lon = float(data[0]['lon'])
                self.get_logger().info(f"[ROUTE] Geocoded '{place_name}' → lat={lat}, lon={lon}")
                return lat, lon
            else:
                self.get_logger().error(f"[ROUTE] No geocoding result for '{place_name}'")
                return None, None
        except Exception as e:
            self.get_logger().error(f"[ROUTE] Geocoding error: {e}")
            return None, None

    def set_route_callback(self, request, response):
        self.waypoints.clear()
        for wp in request.waypoints:
            if isinstance(wp, str):
                lat, lon = self.geocode_place(wp)
                if lat is None:
                    response.success = False
                    response.message = f"Failed to geocode {wp}"
                    return response
            else:
                lat, lon = wp.lat, wp.lon

            x, y = self.transformer.transform(lon, lat)
            local_x = x - self.origin_x
            local_y = y - self.origin_y

            point = Point(x=local_x, y=local_y, z=0.0)
            self.waypoints.append(point)

        self.current_index = 0
        if self.waypoints:
            self.current_target_pub.publish(self.waypoints[0])
            self.debug_pub.publish(self.waypoints[0])

        response.success = True
        response.message = f"Route with {len(self.waypoints)} waypoints set."
        self.get_logger().info(f"[ROUTE] {response.message}")
        return response

    def waypoint_reached_callback(self, msg):
        if not msg.data:
            return
        self.current_index += 1
        if self.current_index < len(self.waypoints):
            pt = self.waypoints[self.current_index]
            self.current_target_pub.publish(pt)
            self.debug_pub.publish(pt)
            self.get_logger().info(f"[ROUTE] Proceeding to waypoint {self.current_index+1}")
        else:
            self.get_logger().info("[ROUTE] Route complete. Destination reached.")

def main(args=None):
    rclpy.init(args=args)
    node = RouteManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
