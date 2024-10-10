import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np


class RealSenseCameraPublisher(Node):
    def __init__(self):
        super().__init__('realsense_camera_publisher')
        self.publisher_rgb = self.create_publisher(Image, 'camera/rgb/image_raw', 10)
        self.publisher_depth = self.create_publisher(Image, 'camera/depth/image_raw', 10)
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()

        # Configure the RealSense camera
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipeline.start(config)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # Convert images to ROS2 messages
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        rgb_msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono16")

        # Publish the images
        self.publisher_rgb.publish(rgb_msg)
        self.publisher_depth.publish(depth_msg)

    def destroy(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    realsense_camera_publisher = RealSenseCameraPublisher()
    rclpy.spin(realsense_camera_publisher)
    realsense_camera_publisher.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
