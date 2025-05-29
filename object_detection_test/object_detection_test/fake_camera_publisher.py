import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class FakeCameraPublisher(Node):
    def __init__(self):
        super().__init__('fake_camera_publisher')
        self.publisher = self.create_publisher(Image, '/zed/image_raw', 10)
        self.bridge = CvBridge()
        self.image = cv2.imread('/home/jetson/ros2_ws/object_detection_test/test_images/plane.jpg')  # CHANGE THE IMAGE PATH
        if self.image is None:
            self.get_logger().error("Image not found or invalid path.")
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 FPS

    def timer_callback(self):
        if self.image is not None:
            msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

