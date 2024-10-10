import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class OpenCVAlgorithmSubscriber(Node):
    def __init__(self):
        super().__init__('opencv_algorithm_subscriber')
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Process the frame (example: converting to grayscale and edge detection)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)

        # Display the processed frame
        cv2.imshow("Processed Frame", edges)
        cv2.waitKey(1)  # Wait for a short time to allow window to refresh

def main(args=None):
    rclpy.init(args=args)
    opencv_algorithm_subscriber = OpenCVAlgorithmSubscriber()
    rclpy.spin(opencv_algorithm_subscriber)
    opencv_algorithm_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
