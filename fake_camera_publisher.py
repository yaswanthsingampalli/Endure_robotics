#This code is for the Video
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

        # Load video file instead of image
        self.cap = cv2.VideoCapture('/home/jetson/ros2_ws/object_detection_test/test_videos/sample.mp4')  # CHANGE THIS PATH

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open video file.")
            return

        fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video ended. Restarting...")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

