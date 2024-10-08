#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')
        
        # Create a publisher for Image messages
        self.publisher_ = self.create_publisher(Image, 'csi_camera/image_raw', 10)
        
        # Initialize the OpenCV bridge
        self.bridge = CvBridge()
        
        # Set the timer callback for periodic image capture and publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        
        # Define the GStreamer pipeline for the CSI camera
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open CSI camera")
            exit(1)
        self.get_logger().info("CSI camera successfully initialized")

    def gstreamer_pipeline(self, capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=30, flip_method=0):
        return (
            f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
            f"format=(string)NV12, framerate=(fraction){framerate}/1 ! nvvidconv flip-method={flip_method} ! "
            f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
            "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
        )

    def timer_callback(self):
        """
        Capture an image from the CSI camera and publish it to the ROS 2 topic.
        """
        ret, frame = self.cap.read()
        if ret:
            # Convert the OpenCV image to a ROS 2 Image message
            image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # Publish the image message
            self.publisher_.publish(image_message)
            self.get_logger().info("Published image")
        else:
            self.get_logger().error("Failed to capture image")

    def destroy_node(self):
        """
        Ensure we release the camera on shutdown.
        """
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down CSI camera node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
