#dummy_origin_publisher.py
import rclpy
from rclpy.node import Node
from airport_navigation_interfaces.msg import GNSSOrigin
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class DummyOriginPublisher(Node):
    def __init__(self):
        super().__init__('dummy_origin_publisher')

        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.pub = self.create_publisher(GNSSOrigin, 'gnss/first_fix', qos_profile)

        # Publish every second continuously
        self.timer = self.create_timer(1.0, self.publish_origin)

    def publish_origin(self):
        msg = GNSSOrigin()
        msg.latitude = 37.78110807505081
        msg.longitude = -97.11376774980187
        msg.altitude = 0.0

        self.get_logger().info(f'📡 Publishing dummy GNSS origin: {msg.latitude}, {msg.longitude}, {msg.altitude}')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyOriginPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
