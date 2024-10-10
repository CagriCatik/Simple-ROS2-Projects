import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CSICameraPublisher(Node):
    def __init__(self):
        super().__init__('csi_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 10 Hz
        self.cap = cv2.VideoCapture(self.get_gstreamer_pipeline(), cv2.CAP_GSTREAMER)

    def get_gstreamer_pipeline(self):
        return ('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! '
                'nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! appsink')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(image_message)
        else:
            self.get_logger().warn('Failed to capture frame')

def main(args=None):
    rclpy.init(args=args)
    csi_camera_publisher = CSICameraPublisher()
    rclpy.spin(csi_camera_publisher)
    csi_camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
