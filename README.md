# Simple-ROS2-Projects

This repository contains multiple workspaces for simple ROS 2 projects, each focusing on specific features or functionalities within the ROS 2 framework. Below is a brief description of each workspace.

## Workspaces

### 1. `ros2_csi_ws`

**CSI Camera ROS 2 Node**:  
This workspace demonstrates the use of CSI (Camera Serial Interface) cameras with ROS 2. It includes nodes for managing camera input and integrating the data into the ROS 2 ecosystem. The project handles CSI camera feeds, providing an interface for capturing images and sending them over ROS 2 topics.

### 2. `ros2_deserialization_ws`

**Custom ROS 2 Messages Using Python Serialization**:  
This workspace is focused on defining and using custom ROS 2 messages with Python serialization. It includes examples of creating custom message types, serializing and deserializing these messages, and sending them between ROS 2 nodes using Python. This is useful for efficiently transmitting custom data in a ROS 2 system.

### 3. `ros2_imu_serial`

**IMU Sensor Data Processing with Serial Communication**:  
This workspace is dedicated to processing data from an IMU sensor (e.g., MPU6050) and sending it over serial communication to ROS 2 nodes. The focus is on reading IMU sensor data, converting it into usable formats, and publishing it as ROS 2 messages for further processing.

### 4. `ros2_lighting_serial`

**Serial Communication for RGB LED Strip Control**:  
This workspace handles sending commands from ROS 2 nodes to an Arduino (or similar microcontroller) over serial communication to control an RGB LED strip. Various lighting modes are implemented and can be triggered via ROS 2 service calls or topic messages.

### 5. `ros2_realsense_ws`

**Intel RealSense Camera Integration**:  
This workspace integrates an Intel RealSense camera with ROS 2. It includes nodes for handling depth and stereo camera data from the RealSense sensor, allowing developers to stream and process 3D camera data in real-time for use in applications like object detection or environmental mapping.

### 6. `ros2_rplidar-c1_ws`

**RPLIDAR Integration with ROS 2**:  
A workspace for integrating RPLIDAR C1 series LiDAR sensors with ROS 2. This workspace provides nodes for capturing and processing 360-degree range data from the LiDAR, publishing it as ROS 2 messages for SLAM (Simultaneous Localization and Mapping) or obstacle avoidance tasks.

### 7. `ros2_srv_ws`

**ROS 2 Services (srv) Tutorial**:  
A workspace focused on understanding and implementing ROS 2 services (`srv`). It includes examples of defining and using custom service types, as well as client-server communication. This project is useful for learning the fundamentals of ROS 2 service communication patterns.

### 8. `ros2_temperature_ws`

**Custom ROS 2 Messages for Simulating Temperature Data**:  
A workspace that simulates temperature data using custom ROS 2 messages. This project focuses on creating custom message types specifically for temperature data and publishing that data through a ROS 2 topic. It's a useful example of sensor data simulation in a ROS 2 environment.

### 9. `ros2_yolo11_detector`

**Object Detection Using YOLOv11 and ROS 2**:  
This workspace integrates the YOLO (You Only Look Once) object detection algorithm (version 11) with ROS 2. It provides a ROS 2 node that processes video streams from a camera (e.g., CSI or RealSense) and detects objects in real-time, publishing the detected objects as ROS 2 messages.

## How to Use

To use any of the workspaces in this repository, follow these steps:

1. Clone the repository:

   ```bash
   git clone <repository-url>
   ```

2. Navigate to the desired workspace, e.g., `ros2_srv_ws`:

   ```bash
   cd ros2_srv_ws
   ```

3. Build the workspace using colcon:

   ```bash
   colcon build
   ```

4. Source the workspace:

   ```bash
   source install/setup.bash
   ```

5. Run the relevant nodes or examples, as detailed in the respective workspace README files.