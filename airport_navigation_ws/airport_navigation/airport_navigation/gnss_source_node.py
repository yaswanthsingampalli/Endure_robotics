#gnss_source_node.py
#!/usr/bin/env python3
# gnss_source_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped, TransformStamped
from airport_navigation_interfaces.msg import GNSSOrigin
import tf2_ros
from pyproj import Transformer
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class GNSSSourceNode(Node):
    def __init__(self):
        super().__init__('gnss_source_node')

        self.gnss_sub = self.create_subscription(NavSatFix, '/fix', self.gnss_callback, 10)
        self.local_pos_pub = self.create_publisher(PointStamped, 'current_position', 10)

        # QoS with transient local to latch last origin message for late subscribers
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.origin_pub = self.create_publisher(GNSSOrigin, 'gnss/origin', qos_profile)
        self.origin_fixed_pub = self.create_publisher(GNSSOrigin, 'gnss/first_fix', qos_profile)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.frame_id = "map"
        self.child_frame_id = "base_link"
        self.origin_set = False

        self.origin_lat = 0.0
        self.origin_lon = 0.0
        self.origin_alt = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.transformer = None
        self.get_logger().info("[GNSS] Waiting for first GNSS fix...")

        # Timer to repeatedly publish first fix (once set)
        self.publish_origin_timer = self.create_timer(1.0, self.publish_origin)

    def gnss_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warn("[GNSS] No valid fix yet")
            return

        lon, lat, alt = msg.longitude, msg.latitude, msg.altitude

        if not self.origin_set:
            self.origin_lat = lat
            self.origin_lon = lon
            self.origin_alt = alt

            # 🔁 Dynamic UTM zone based on origin
            utm_zone = int((lon + 180) / 6) + 1
            is_northern = lat >= 0
            epsg_code = 32600 + utm_zone if is_northern else 32700 + utm_zone

            self.transformer = Transformer.from_crs("epsg:4326", f"epsg:{epsg_code}", always_xy=True)
            self.origin_x, self.origin_y = self.transformer.transform(lon, lat)
            self.origin_set = True

            origin_msg = GNSSOrigin()
            origin_msg.latitude = self.origin_lat
            origin_msg.longitude = self.origin_lon
            origin_msg.altitude = self.origin_alt

            self.origin_pub.publish(origin_msg)
            self.origin_fixed_msg = origin_msg
            self.get_logger().info(
                f"[GNSS] Origin set: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}, using EPSG:{epsg_code}"
            )

        # publish local position continuously
        x, y = self.transformer.transform(lon, lat)
        local_x = x - self.origin_x
        local_y = y - self.origin_y
        local_z = alt - self.origin_alt

        pt_msg = PointStamped()
        pt_msg.header.stamp = self.get_clock().now().to_msg()
        pt_msg.header.frame_id = self.frame_id
        pt_msg.point.x = local_x
        pt_msg.point.y = local_y
        pt_msg.point.z = local_z
        self.local_pos_pub.publish(pt_msg)

        # Broadcast TF transform
        t = TransformStamped()
        t.header = pt_msg.header
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = local_x
        t.transform.translation.y = local_y
        t.transform.translation.z = local_z
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def publish_origin(self):
        # Re-publish first fix continuously for late subscribers
        if self.origin_set and hasattr(self, 'origin_fixed_msg'):
            self.origin_fixed_pub.publish(self.origin_fixed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GNSSSourceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
