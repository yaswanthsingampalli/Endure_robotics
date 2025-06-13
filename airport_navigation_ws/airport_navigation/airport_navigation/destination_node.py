#destination_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from airport_navigation_interfaces.srv import SetDestination
import requests

class DestinationNode(Node):
    def __init__(self):
        super().__init__('destination_node')
        self.publisher_ = self.create_publisher(Point, 'destination_position', 10)
        self.debug_pub = self.create_publisher(Point, 'debug/destination_position', 10)
        self.srv = self.create_service(SetDestination, 'set_destination', self.set_destination_callback)
        self.destination = None
        self.timer = self.create_timer(2.0, self.publish_destination)

        self.get_logger().info('[DEST] Destination node is ready.')

    def publish_destination(self):
        if self.destination is not None:
            self.publisher_.publish(self.destination)
            self.debug_pub.publish(self.destination)

    def set_destination_callback(self, request, response):
        try:
            if request.place_name:
                self.get_logger().info(f"[DEST] Geocoding: {request.place_name}")
                coords = self.geocode_place_name(request.place_name)
                if coords is None:
                    response.success = False
                    response.message = "Failed to geocode destination."
                    return response
                x, y, z = coords
            else:
                x, y, z = request.x, request.y, request.z

            self.destination = Point(x=x, y=y, z=z)
            response.success = True
            response.message = f"Destination set to ({x:.2f}, {y:.2f}, {z:.2f})"
            self.get_logger().info(f"[DEST] {response.message}")
            return response

        except Exception as e:
            self.get_logger().error(f"[DEST] Error: {str(e)}")
            response.success = False
            response.message = "Internal error occurred."
            return response

    def geocode_place_name(self, place_name):
        try:
            url = 'https://nominatim.openstreetmap.org/search'
            params = {'q': place_name, 'format': 'json', 'limit': 1}
            headers = {'User-Agent': 'ros2-robot'}
            resp = requests.get(url, params=params, headers=headers)
            data = resp.json()
            if not data:
                return None
            lat = float(data[0]['lat'])
            lon = float(data[0]['lon'])
            return lon, lat, 0.0
        except Exception as e:
            self.get_logger().error(f"[DEST] Geocoding failed: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = DestinationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
