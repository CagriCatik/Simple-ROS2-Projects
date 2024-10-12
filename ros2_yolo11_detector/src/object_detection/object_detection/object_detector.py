import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        # Load YOLO model from the provided "yolo11s.pt" file
        model_path = 'src/check_points/yolo11s.pt'  # Adjust the path if needed
        self.model = torch.load(model_path)
        self.model.eval()  # Set the model to evaluation mode

    def listener_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Prepare frame for YOLO model (e.g., convert to Tensor)
        img_tensor = torch.from_numpy(frame).float().permute(2, 0, 1).unsqueeze(0)  # Adjust as necessary

        # YOLO Inference
        with torch.no_grad():
            results = self.model(img_tensor)
        
        # Assuming the model output follows a similar format (bounding boxes, classes, confidence scores)
        detections = results[0]  # Adjust this based on your model's output format
        
        # Draw bounding boxes on the frame
        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            label = f'{int(cls)} {conf:.2f}'
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Optionally display the result for testing
        cv2.imshow('YOLO Object Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
