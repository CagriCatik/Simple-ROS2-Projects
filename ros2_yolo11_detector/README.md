# ROS2 YOLO11 Object Detection Package

This ROS 2 package (`object_detection`) enables real-time object detection using a CSI camera and a custom YOLO11 model (`yolo11s.pt`). The package consists of two main ROS 2 nodes: one that publishes the video stream from a CSI camera and another that subscribes to the stream, performs object detection using the YOLO model, and visualizes the results.

## Table of Contents

- [ROS2 YOLO11 Object Detection Package](#ros2-yolo11-object-detection-package)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
    - [1. Clone the Repository](#1-clone-the-repository)
    - [2. Install Python Dependencies](#2-install-python-dependencies)
    - [3. Build the Package](#3-build-the-package)
    - [4. Source the Workspace](#4-source-the-workspace)
  - [Usage](#usage)
    - [1. Run the CSI Camera Publisher](#1-run-the-csi-camera-publisher)
    - [2. Run the Object Detector](#2-run-the-object-detector)
    - [3. Visualization](#3-visualization)
  - [Nodes](#nodes)
    - [CSI Camera Publisher](#csi-camera-publisher)
    - [Object Detector](#object-detector)
  - [Dependencies](#dependencies)
  - [Notes](#notes)

## Installation

### 1. Clone the Repository

First, clone the repository into your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone <repository_url> ROS2_YOLO11_DETECTOR
```

### 2. Install Python Dependencies

Ensure you have the necessary Python dependencies by installing the packages listed in `requirements.txt`:

```bash
cd ~/ros2_ws/src/ROS2_YOLO11_DETECTOR
pip install -r requirements.txt
```

### 3. Build the Package

Go to your workspace root and build the package using `colcon`:

```bash
cd ~/ros2_ws
colcon build
```

### 4. Source the Workspace

After building, source the workspace to include the package in your environment:

```bash
source install/setup.bash
```

## Usage

This package contains two ROS 2 nodes: a CSI camera publisher and an object detector.

### 1. Run the CSI Camera Publisher

The camera publisher node captures video from the CSI camera and publishes it on the `/camera/image_raw` topic.

In one terminal, run:

```bash
ros2 run object_detection csi_camera_publisher
```

### 2. Run the Object Detector

The object detector node subscribes to the `/camera/image_raw` topic, performs object detection using the `yolo11s.pt` model, and visualizes the results by drawing bounding boxes on the detected objects.

In a separate terminal, run:

```bash
ros2 run object_detection object_detector
```

### 3. Visualization

You can visualize the output either using the OpenCV window already provided by the object detection node or using ROS tools like `rqt_image_view` to view the video feed.

For `rqt_image_view`, run:

```bash
rqt_image_view
```

Then select the `/camera/image_raw` topic to display the video stream.

## Nodes

### CSI Camera Publisher

The CSI Camera Publisher node captures the video feed from the connected CSI camera and publishes it as ROS 2 `Image` messages on the `/camera/image_raw` topic.

- **Node Name**: `csi_camera_publisher`
- **Published Topic**: `/camera/image_raw`
- **File**: `src/object_detection/object_detection/csi_camera_publisher.py`

### Object Detector

The Object Detector node subscribes to the `/camera/image_raw` topic, runs the YOLO11 model on the frames, and draws bounding boxes around detected objects. It visualizes the results either by publishing the output or displaying it in an OpenCV window.

- **Node Name**: `object_detector`
- **Subscribed Topic**: `/camera/image_raw`
- **File**: `src/object_detection/object_detection/object_detector.py`

## Dependencies

All dependencies required by the project are listed in `requirements.txt`:

- `torch` (for PyTorch-based YOLO model loading and inference)
- `opencv-python` (for camera input and display)
- `cv_bridge` (for converting ROS messages to OpenCV format)
- `rclpy` (Python API for ROS 2)
- `sensor_msgs` (ROS 2 messages for sensor data, such as images)

To install these dependencies, run:

```bash
pip install -r requirements.txt
```

## Notes

1. **Model Path**: The `yolo11s.pt` model is located in `src/check_points/yolo11s.pt`. Make sure this file is available and that the path is correctly set in the object detection script.
   
2. **ROS 2 Version**: This package assumes that you're using ROS 2 Humble distribution or later. Make sure your ROS environment is properly sourced before running the nodes.

3. **CSI Camera Configuration**: The `csi_camera_publisher.py` file is configured to use the camera index `0`. If you have multiple cameras, adjust the camera index as needed.