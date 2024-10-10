import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class StereoDepthSubscriber(Node):
    def __init__(self):
        super().__init__('stereo_depth_subscriber')
        self.bridge = CvBridge()

        # Subscribing to RGB and Depth topics
        self.subscription_rgb = self.create_subscription(Image, 'camera/rgb/image_raw', self.rgb_callback, 10)
        self.subscription_depth = self.create_subscription(Image, 'camera/depth/image_raw', self.depth_callback, 10)

    def rgb_callback(self, msg):
        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("RGB Image", rgb_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')

        # Normalize depth for visualization
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imshow("Depth Image", depth_image_normalized)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    stereo_depth_subscriber = StereoDepthSubscriber()
    rclpy.spin(stereo_depth_subscriber)
    stereo_depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
