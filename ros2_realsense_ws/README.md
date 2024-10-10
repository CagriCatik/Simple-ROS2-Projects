# ROS 2 RealSense Camera Publisher and Depth/ Stereo Subscriber

## Overview

This project demonstrates how to use an **Intel RealSense camera** in a ROS 2 environment to stream RGB and depth data, and how to process that data using a subscriber node. The project consists of two main components:

1. **RealSense Camera Publisher**: Streams RGB and depth data from the Intel RealSense camera and publishes them as ROS 2 messages.
2. **Stereo Depth Subscriber**: Subscribes to the RGB and depth data, processes it using OpenCV, and displays the processed data, including depth visualization.

The setup leverages ROS 2's messaging system for real-time data exchange between the nodes, while OpenCV is used for image processing tasks. This project is ideal for applications involving robotics, computer vision, and 3D sensing.

---

## Features

- **RGB and Depth Data Streaming**: Real-time streaming of both RGB and depth data from an Intel RealSense camera.
- **Depth and Stereo Processing**: Processes depth images and RGB images using OpenCV to visualize and handle depth data for computer vision tasks.
- **ROS 2 Integration**: Leverages the ROS 2 ecosystem to enable modular communication between camera hardware and the image processing algorithms.
- **Extensible Framework**: The project can be extended to include more complex image processing tasks, such as object detection, 3D mapping, or SLAM.

---

## Prerequisites

Before you begin, ensure that your environment meets the following requirements:

### Software Requirements

- **ROS 2 (Humble or Foxy)**: Ensure ROS 2 is installed and set up correctly. Follow the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) if needed.
- **Intel RealSense SDK**: Install the Intel RealSense SDK to interact with the RealSense camera. You can find installation instructions on the [Intel RealSense SDK page](https://github.com/IntelRealSense/librealsense).
- **OpenCV**: OpenCV is required for image processing. Install OpenCV through your package manager or using pip:

  ```bash
  sudo apt install python3-opencv
  ```

- **RealSense ROS 2 Wrapper**: The official Intel RealSense ROS 2 wrapper is required to integrate the RealSense camera with ROS 2. Install it via the following command:

  ```bash
  sudo apt install ros-humble-realsense2-camera
  ```

### Hardware Requirements

- **Intel RealSense Camera**: A compatible Intel RealSense camera such as the **D435**, **D415**, or **D455** models.
- **Computer with ROS 2 Installed**: Ensure that your computer is running ROS 2 and has enough performance to handle real-time video and depth stream processing.

---

## Project Setup

### 1. Create the ROS 2 Workspace

First, create a workspace where you will store the project and all its packages. You can create a new workspace or use an existing one.

```bash
mkdir -p ~/ros2_realsense_ws/src
cd ~/ros2_realsense_ws
```

### 2. Clone or Create the Project Packages

Inside the `src` directory, create two packages:

1. **RealSense Camera Publisher**: Responsible for streaming RGB and depth data from the camera.
2. **Stereo Depth Subscriber**: Subscribes to the camera's streams and processes the data.

Ensure that the package structure is properly configured for both publisher and subscriber nodes. The Python code should be organized in Python modules, and dependencies like `rclpy`, `cv_bridge`, and `sensor_msgs` should be correctly specified in each package.

### 3. Install Dependencies

The project relies on a few critical dependencies that must be installed in your environment:

- **rclpy**: ROS 2 Python client library.
- **sensor_msgs**: ROS 2 message types for handling images.
- **cv_bridge**: A ROS 2 library to convert between ROS image messages and OpenCV images.
  
Install any missing dependencies using your package manager:

```bash
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-cv-bridge
```

### 4. Build the Workspace

Once the packages are set up, you need to build the workspace using `colcon`:

```bash
cd ~/ros2_realsense_ws
colcon build --packages-select realsense_camera_publisher stereo_depth_subscriber
```

This command will compile the nodes in both packages and prepare them for execution. Ensure that there are no errors during the build process.

### 5. Source the Workspace

Before running any ROS 2 nodes, ensure that your environment is set up correctly by sourcing the workspace:

```bash
source install/setup.bash
```

This command allows ROS 2 to recognize the newly built packages and their nodes.

---

## Running the Nodes

### 1. Run the RealSense Camera Publisher

The first step is to start the RealSense camera publisher node. This node will connect to the Intel RealSense camera and start streaming both RGB and depth images to specific ROS 2 topics.

In a new terminal, make sure your workspace is sourced and run the publisher node:

```bash
ros2 run realsense_camera_publisher realsense_camera_node
```

This node will begin publishing RGB data on `/camera/rgb/image_raw` and depth data on `/camera/depth/image_raw`.

### 2. Run the Stereo Depth Subscriber

Once the camera publisher is streaming data, you can start the subscriber node, which will process the data. The subscriber node will receive the RGB and depth images and display them, using OpenCV for depth visualization.

Open another terminal, source the workspace, and run the subscriber node:

```bash
ros2 run stereo_depth_subscriber stereo_depth_node
```

This node will listen to the image topics and process the incoming data.

---

## Troubleshooting

### Package Not Found

If you encounter the error `Package 'stereo_depth_subscriber' not found` or similar:

- Ensure you have sourced the workspace correctly (`source install/setup.bash`).
- Make sure the package was built successfully by running `colcon build --packages-select <package-name>`.
- Verify that the package is listed using `ros2 pkg list`.

### Camera Not Detected

If the Intel RealSense camera is not detected or fails to stream:

- Check that the camera is properly connected to the computer.
- Ensure that the RealSense SDK and the ROS 2 RealSense wrapper are installed and up-to-date.
- Run `realsense-viewer` to verify that the camera is functioning correctly.

### OpenCV Display Issues

If the OpenCV windows are not appearing or the frames are not displayed:

- Ensure OpenCV is installed properly in your environment.
- Check that the `cv_bridge` package is properly installed and working.
- Verify that the correct topics are being subscribed to and that the frames are being processed.

---

## Extending the Project

This project serves as a foundation for more advanced image processing and computer vision tasks. Here are some ways you can extend the functionality:

- **Object Detection**: Implement object detection algorithms (such as YOLO or SSD) to detect and track objects in the camera's field of view.
- **3D Mapping**: Use the depth data from the RealSense camera to build a 3D map of the environment or perform SLAM.
- **Robot Navigation**: Integrate the camera streams with a robot control system to enable autonomous navigation and obstacle avoidance.
- **Point Cloud Generation**: Generate point clouds from the depth data and visualize them using RViz or other 3D visualization tools.

---

## Future Improvements

- **Data Recording**: Add the ability to record and playback RGB and depth data using ROS 2 bag files.
- **Real-Time Processing**: Integrate hardware acceleration for real-time image processing and object recognition.
