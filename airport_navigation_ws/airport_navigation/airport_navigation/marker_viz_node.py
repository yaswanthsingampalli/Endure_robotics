#!/usr/bin/env python3
#marker_viz_node.py
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PointStamped

class MarkerVizNode(Node):
    def __init__(self):
        super().__init__('marker_viz_node')
        self.get_logger().info('[RVIZ MARKER] Enhanced Visualization Node Started.')

        self.waypoints = []
        self.destination = None
        self.current_position = None
        self.trail_history = []

        self.marker_pub = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        self.create_subscription(PointStamped, '/waypoints_enu', self.waypoint_callback, 10)
        self.create_subscription(PointStamped, '/destination_position', self.destination_callback, 10)
        self.create_subscription(PointStamped, '/current_position', self.current_position_callback, 10)

        self.timer = self.create_timer(1.0, self.publish_markers)

    def waypoint_callback(self, msg: PointStamped):
        # Add/update waypoint based on coordinates or add simple (no name in PointStamped)
        # For simplicity, just append new waypoints (could add logic to prevent duplicates)
        if not any(abs(wp.point.x - msg.point.x) < 0.001 and abs(wp.point.y - msg.point.y) < 0.001 for wp in self.waypoints):
            self.waypoints.append(msg)
            self.get_logger().info(f"📍 Waypoint received at x={msg.point.x:.2f}, y={msg.point.y:.2f}")

    def destination_callback(self, msg: PointStamped):
        self.destination = msg
        self.get_logger().info(f"🎯 Destination received at x={msg.point.x:.2f}, y={msg.point.y:.2f}")

    def current_position_callback(self, msg: PointStamped):
        self.current_position = msg
        self.trail_history.append(msg.point)
        if len(self.trail_history) > 100:
            self.trail_history.pop(0)

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Waypoints as Green Spheres
        for i, wp in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = wp.point.x
            m.pose.position.y = wp.point.y
            m.pose.position.z = 0.2
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.4
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime.sec = 2
            marker_array.markers.append(m)

        # Route line connecting waypoints
        if len(self.waypoints) > 1:
            line = Marker()
            line.header.frame_id = "map"
            line.header.stamp = now
            line.id = 999
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.1
            line.color.r = 0.0
            line.color.g = 0.5
            line.color.b = 1.0
            line.color.a = 0.8
            line.lifetime.sec = 2
            line.points = [Point(x=wp.point.x, y=wp.point.y, z=0.1) for wp in self.waypoints]
            marker_array.markers.append(line)

        # Destination as Red Cube
        if self.destination:
            dest = Marker()
            dest.header.frame_id = "map"
            dest.header.stamp = now
            dest.id = 2000
            dest.type = Marker.CUBE
            dest.action = Marker.ADD
            dest.pose.position.x = self.destination.point.x
            dest.pose.position.y = self.destination.point.y
            dest.pose.position.z = 0.2
            dest.scale.x = 0.5
            dest.scale.y = 0.5
            dest.scale.z = 0.5
            dest.color.r = 1.0
            dest.color.g = 0.0
            dest.color.b = 0.0
            dest.color.a = 0.9
            dest.lifetime.sec = 2
            marker_array.markers.append(dest)

        # Current robot position as white sphere
        if self.current_position:
            curr = Marker()
            curr.header.frame_id = "map"
            curr.header.stamp = now
            curr.id = 3000
            curr.type = Marker.SPHERE
            curr.action = Marker.ADD
            curr.pose.position = self.current_position.point
            curr.scale.x = 0.3
            curr.scale.y = 0.3
            curr.scale.z = 0.3
            curr.color.r = 1.0
            curr.color.g = 1.0
            curr.color.b = 1.0
            curr.color.a = 1.0
            curr.lifetime.sec = 1
            marker_array.markers.append(curr)

        # Trail history as faded purple dots
        if len(self.trail_history) > 1:
            trail = Marker()
            trail.header.frame_id = "map"
            trail.header.stamp = now
            trail.id = 4000
            trail.type = Marker.SPHERE_LIST
            trail.action = Marker.ADD
            trail.scale.x = 0.15
            trail.scale.y = 0.15
            trail.scale.z = 0.15
            trail.color.r = 1.0
            trail.color.g = 0.0
            trail.color.b = 1.0
            trail.color.a = 0.4
            trail.points = self.trail_history
            trail.lifetime.sec = 2
            marker_array.markers.append(trail)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerVizNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
