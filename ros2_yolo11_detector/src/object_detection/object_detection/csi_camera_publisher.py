import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CSICameraPublisher(Node):
    def __init__(self):
        super().__init__('csi_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # Make sure this points to the correct camera index

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher_.publish(img_msg)
            self.get_logger().info('Publishing camera frame')

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
