#!/usr/bin/env python3
#pose_publisher_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray, Pose
from std_msgs.msg import Header
from threading import Lock

class PosePublisherNode(Node):
    def __init__(self):
        super().__init__('pose_publisher_node')

        self.frame_id = "map"

        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.waypoints_pose_pub = self.create_publisher(PoseArray, '/waypoints_pose', 10)

        self.current_position_sub = self.create_subscription(
            PointStamped,
            '/current_position',
            self.current_position_callback,
            10
        )
        self.destination_position_sub = self.create_subscription(
            PointStamped,
            '/destination_position',
            self.destination_position_callback,
            10
        )
        self.waypoints_sub = self.create_subscription(
            PointStamped,
            '/waypoints_enu',
            self.waypoints_callback,
            10
        )

        self.waypoints = []
        self.waypoints_lock = Lock()

        self.get_logger().info("Pose Publisher Node started and subscribing to ENU topics.")

        # Optional: Timer to publish PoseArray periodically (e.g., 1 Hz)
        self.timer = self.create_timer(1.0, self.publish_waypoints_posearray)

    def current_position_callback(self, msg: PointStamped):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = msg.point.x
        pose_msg.pose.position.y = msg.point.y
        pose_msg.pose.position.z = msg.point.z
        pose_msg.pose.orientation.w = 1.0  # Identity orientation
        self.current_pose_pub.publish(pose_msg)
        self.get_logger().debug(f"Published current_pose at ({pose_msg.pose.position.x}, {pose_msg.pose.position.y})")

    def destination_position_callback(self, msg: PointStamped):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = msg.point.x
        pose_msg.pose.position.y = msg.point.y
        pose_msg.pose.position.z = msg.point.z
        pose_msg.pose.orientation.w = 1.0
        self.goal_pose_pub.publish(pose_msg)
        self.get_logger().debug(f"Published goal_pose at ({pose_msg.pose.position.x}, {pose_msg.pose.position.y})")

    def waypoints_callback(self, msg: PointStamped):
        with self.waypoints_lock:
            # We'll store all unique waypoints (based on coordinates)
            found = False
            for wp in self.waypoints:
                if (abs(wp.point.x - msg.point.x) < 1e-3 and
                    abs(wp.point.y - msg.point.y) < 1e-3 and
                    abs(wp.point.z - msg.point.z) < 1e-3):
                    found = True
                    break
            if not found:
                self.waypoints.append(msg)
                self.get_logger().info(f"Added waypoint: ({msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f})")

    def publish_waypoints_posearray(self):
        with self.waypoints_lock:
            if not self.waypoints:
                return

            pose_array = PoseArray()
            pose_array.header = Header()
            pose_array.header.stamp = self.get_clock().now().to_msg()
            pose_array.header.frame_id = self.frame_id

            for wp in self.waypoints:
                pose = Pose()
                pose.position.x = wp.point.x
                pose.position.y = wp.point.y
                pose.position.z = wp.point.z
                pose.orientation.w = 1.0
                pose_array.poses.append(pose)

            self.waypoints_pose_pub.publish(pose_array)
            self.get_logger().debug(f"Published PoseArray with {len(self.waypoints)} waypoints.")

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
