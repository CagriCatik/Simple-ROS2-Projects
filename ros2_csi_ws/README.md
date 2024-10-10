# ROS 2 CSI Camera Publisher and OpenCV Subscriber

## Overview

This project demonstrates a simple yet powerful example of a ROS 2 application, consisting of two nodes:

- **Node 1: `csi_camera_publisher`**  
  A ROS 2 publisher node that streams video data from a CSI camera (e.g., a Jetson Nano connected camera) using OpenCV and GStreamer pipelines. The video frames are published as ROS 2 messages to a topic.

- **Node 2: `opencv_subscriber`**  
  A ROS 2 subscriber node that receives the video stream from the publisher and processes it using OpenCV. In this example, the processing consists of converting the video stream to grayscale and applying edge detection.

This example serves as a foundation for ROS 2 projects that involve video streaming and real-time image processing.

---

## Features

- **CSI Camera Streaming**: Leverages GStreamer for efficient video capture from CSI cameras.
- **ROS 2 Integration**: Publishes and subscribes to ROS 2 topics, providing a modular and scalable architecture.
- **OpenCV Processing**: Applies basic image processing (grayscale conversion and edge detection) to the video stream.

---

## Prerequisites

Before setting up the project, ensure you have the following dependencies installed:

- **ROS 2 (Humble or Foxy)**: Installation instructions can be found in the [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).
- **Python 3**: Ensure you have Python 3 installed.
- **OpenCV**: Install OpenCV for Python:
  
  ```bash
  sudo apt install python3-opencv
  ```

- **GStreamer**: Ensure GStreamer is installed for handling video streams from the CSI camera:
  
  ```bash
  sudo apt install gstreamer1.0-tools
  ```

- **CvBridge**: The `cv_bridge` package is required for converting ROS 2 Image messages to OpenCV image formats. Install it with:
  
  ```bash
  sudo apt install ros-humble-cv-bridge
  ```

---

## Project Setup

### 1. Clone the Repository

Clone the project repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-repository-url/ros2_csi_camera_opencv.git
```

### 2. Build the Project

Once the project is cloned, build the workspace using `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select csi_camera_publisher opencv_subscriber
```

### 3. Source the Workspace

Before running the nodes, source your workspace:

```bash
source install/setup.bash
```

Ensure you source this file in every terminal where you plan to use ROS 2 commands.

---

## Nodes

### 1. CSI Camera Publisher

This node captures video frames from the CSI camera using GStreamer and publishes the frames as ROS 2 messages to the topic `camera/image_raw`.

**Node:** `csi_camera_publisher`  
**Published Topic:** `/camera/image_raw`

#### Running the CSI Camera Publisher Node

Run the CSI camera publisher node:

```bash
ros2 run csi_camera_publisher csi_camera_node
```

If the camera is correctly connected, the node will begin publishing video frames at a frequency of 10 Hz.

---

### 2. OpenCV Algorithm Subscriber

This node subscribes to the video stream published by the `csi_camera_publisher` node. It processes the incoming video frames using OpenCV by applying grayscale conversion and edge detection (`Canny` filter). The processed frames are displayed using OpenCV's `imshow()`.

**Node:** `opencv_algorithm_subscriber`  
**Subscribed Topic:** `/camera/image_raw`

#### Running the OpenCV Subscriber Node

Run the OpenCV subscriber node:

```bash
ros2 run opencv_subscriber opencv_algorithm_node
```

This node will display the processed video frames in a window.

---

## Directory Structure

```bash
ros2_csi_ws/
├── src/
│   ├── csi_camera_publisher/
│   │   ├── csi_camera_publisher/
│   │   │   ├── __init__.py
│   │   │   ├── csi_camera_node.py
│   │   ├── package.xml
│   │   ├── setup.py
│   ├── opencv_subscriber/
│   │   ├── opencv_subscriber/
│   │   │   ├── __init__.py
│   │   │   ├── opencv_algorithm_node.py
│   │   ├── package.xml
│   │   ├── setup.py
```

- **`csi_camera_node.py`**: Implements the CSI camera publisher.
- **`opencv_algorithm_node.py`**: Implements the OpenCV processing subscriber.

---

## Customizing the GStreamer Pipeline

The GStreamer pipeline can be modified in `csi_camera_node.py` to adjust the camera resolution, framerate, or other parameters. The pipeline used in this example:

```python
def get_gstreamer_pipeline():
    return ('nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction)30/1 ! '
            'nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! appsink')
```

You can modify the resolution, flip method, or framerate to suit your camera setup.

---

## Troubleshooting

### 1. **Package Not Found**

If you receive an error like `Package 'csi_camera_publisher' not found`, make sure that:

- The package is correctly built (`colcon build` completes successfully).
- The workspace is sourced (`source install/setup.bash`).

### 2. **Camera Not Detected**

If the CSI camera is not detected, ensure:

- The camera is properly connected to the device (e.g., Jetson Nano).
- GStreamer is correctly installed and configured.

### 3. **OpenCV Window Not Displaying**

If the OpenCV window does not display the processed video:

- Ensure that `cv2.imshow()` is called and that `cv2.waitKey(1)` is used to refresh the display.

---

## Future Enhancements

- **Add More Image Processing Algorithms**: Expand the subscriber node to perform more advanced image processing techniques such as object detection or tracking.
- **Support for Multiple Camera Streams**: Modify the publisher node to handle multiple CSI cameras and stream data simultaneously.
- **Integrate with Other ROS 2 Nodes**: Extend this project to work with other ROS 2 packages such as SLAM, navigation, or machine learning.
