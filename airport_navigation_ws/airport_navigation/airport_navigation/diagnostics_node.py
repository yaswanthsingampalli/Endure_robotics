# diagnostics_node.py
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__('diagnostics_node')
        self.publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.timer = self.create_timer(2.0, self.publish_diagnostics)

    def publish_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "System Health"
        status.level = DiagnosticStatus.OK
        status.message = "All systems nominal"
        status.values = [
            KeyValue(key="CPU Temp", value="45 C"),
            KeyValue(key="Battery", value="12.6V"),
            KeyValue(key="GNSS Lock", value="True"),
            KeyValue(key="Waypoints Loaded", value="Yes"),
        ]

        msg.status.append(status)
        self.publisher.publish(msg)
        self.get_logger().debug("[DIAGNOSTICS] Published system status.")

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
