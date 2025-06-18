#!/usr/bin/env python3
#gnss_source_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_ros
from pyproj import Transformer

class GNSSSourceNode(Node):
    def __init__(self):
        super().__init__('gnss_source_node')
        self.gnss_sub = self.create_subscription(NavSatFix, '/fix', self.gnss_callback, 10)
        self.local_pos_pub = self.create_publisher(PointStamped, 'current_position', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Coordinate transformer WGS84 (lat/lon) -> UTM zone auto
        self.transformer = Transformer.from_crs("epsg:4326", "epsg:32614", always_xy=True)  # adapt zone or make dynamic

        self.frame_id = "map"  # global fixed frame
        self.child_frame_id = "base_link"  # robot base
        self.origin_set = False
        self.origin_lat = None
        self.origin_lon = None
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.get_logger().info("[GNSS] Waiting for first GNSS fix...")

    def gnss_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warn("[GNSS] No valid fix yet")
            return

        lon, lat = msg.longitude, msg.latitude

        if not self.origin_set:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_x, self.origin_y = self.transformer.transform(self.origin_lon, self.origin_lat)
            self.origin_set = True
            self.get_logger().info(f"[GNSS] Origin set at lat:{self.origin_lat:.6f} lon:{self.origin_lon:.6f}")

        # Convert to UTM
        x, y = self.transformer.transform(lon, lat)

        # Convert to local ENU relative to origin
        local_x = x - self.origin_x
        local_y = y - self.origin_y

        # Publish position as PointStamped
        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = self.frame_id
        pt_msg.point.x = local_x
        pt_msg.point.y = local_y
        pt_msg.point.z = msg.altitude  # altitude from GNSS fix

        self.local_pos_pub.publish(pt_msg)

        # Broadcast TF from map -> base_link
        t = TransformStamped()
        t.header.stamp = pt_msg.header.stamp
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = local_x
        t.transform.translation.y = local_y
        t.transform.translation.z = msg.altitude
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GNSSSourceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
