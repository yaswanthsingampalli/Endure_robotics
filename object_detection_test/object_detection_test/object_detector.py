from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # or yolov8s.pt if you want better accuracy
        self.sub = self.create_subscription(Image, '/zed/image_raw', self.image_callback, 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Run model inference with confidence threshold and NMS IoU threshold set
            results = self.model(cv_image, conf=0.5, iou=0.45)[0]

            filtered_boxes = []
            filtered_scores = []
            filtered_classes = []

            for det in results.boxes:
                conf = det.conf[0].item()
                if conf >= 0.5:
                    xyxy = det.xyxy[0].cpu().numpy()
                    cls = int(det.cls[0].item())
                    filtered_boxes.append(xyxy)
                    filtered_scores.append(conf)
                    filtered_classes.append(cls)

            labels = [self.model.names[i] for i in filtered_classes]
            self.get_logger().info(f"Predicted labels: {labels}")

            # Draw bounding boxes and labels
            for box, conf, cls in zip(filtered_boxes, filtered_scores, filtered_classes):
                x1, y1, x2, y2 = box.astype(int)
                label = f"{self.model.names[cls]} {conf:.2f}"
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, label, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow("Detections", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

