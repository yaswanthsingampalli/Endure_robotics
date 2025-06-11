#!/usr/bin/env python3
#destination_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from airport_navigation_interfaces.srv import SetDestination

class DestinationNode(Node):
    def __init__(self):
        super().__init__('destination_node')
        self.publisher_ = self.create_publisher(Point, 'destination_position', 10)
        self.destination = Point(x=0.0, y=0.0, z=0.0)
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.srv = self.create_service(SetDestination, 'set_destination', self.set_destination_callback)

    def timer_callback(self):
        self.publisher_.publish(self.destination)

    def set_destination_callback(self, request, response):
        self.destination.x = request.x
        self.destination.y = request.y
        self.destination.z = request.z
        response.success = True
        response.message = f"Destination updated to x={request.x}, y={request.y}, z={request.z}"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DestinationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

