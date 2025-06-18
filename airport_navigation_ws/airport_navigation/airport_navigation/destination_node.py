#destination_node.py
import rclpy
from rclpy.node import Node
from airport_navigation_interfaces.srv import SetDestination
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import pyproj

class DestinationNode(Node):
    def __init__(self):
        super().__init__('destination_node')
        self.declare_parameter('utm_zone', 14)
        self.utm_zone = self.get_parameter('utm_zone').get_parameter_value().integer_value

        self.destination_pub = self.create_publisher(PointStamped, 'destination_position', 10)
        self.debug_pub = self.create_publisher(PointStamped, 'debug/destination_position', 10)
        self.srv = self.create_service(SetDestination, 'set_destination', self.handle_set_destination)

    def handle_set_destination(self, request, response):
        lat = request.y  # y is latitude
        lon = request.x  # x is longitude
        alt = request.z

        utm_easting, utm_northing = self.gps_to_utm(lat, lon)

        msg = PointStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = utm_easting
        msg.point.y = utm_northing
        msg.point.z = alt

        self.destination_pub.publish(msg)
        self.debug_pub.publish(msg)

        response.success = True
        response.message = f"✅ Destination set to lat={lat}, lon={lon}, alt={alt}"
        self.get_logger().info(response.message)
        return response

    def gps_to_utm(self, lat, lon):
        utm_proj = pyproj.Proj(proj='utm', zone=self.utm_zone, ellps='WGS84')
        return utm_proj(lon, lat)  # lon, lat order

def main(args=None):
    rclpy.init(args=args)
    node = DestinationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
