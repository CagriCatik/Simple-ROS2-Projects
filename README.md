# Simple-ROS2-Projects

This repository contains multiple workspaces for simple ROS 2 projects, each focusing on specific features or functionalities within the ROS 2 framework. Below is a brief description of each workspace.

## Workspaces

### 1. `ros2_csi_ws`

**CSI Camera ROS 2 Node**:  
This workspace demonstrates the use of CSI (Camera Serial Interface) cameras with ROS 2. It includes nodes for managing camera input and integrating the data into the ROS 2 ecosystem. The project handles CSI camera feeds, providing an interface for capturing images and sending them over ROS 2 topics.

### 2. `ros2_deserialization_ws`

**Custom ROS 2 Messages Using Python Serialization**:  
This workspace is focused on defining and using custom ROS 2 messages with Python serialization. It includes examples of creating custom message types, serializing and deserializing these messages, and sending them between ROS 2 nodes using Python. This is useful for efficiently transmitting custom data in a ROS 2 system.

### 3. `ros2_temperature_ws`

**Custom ROS 2 Messages for Simulating Temperature Data**:  
A workspace that simulates temperature data using custom ROS 2 messages. This project focuses on creating custom message types specifically for temperature data and publishing that data through a ROS 2 topic. It's a useful example of sensor data simulation in a ROS 2 environment.

### 4. `ros2_srv_ws`

A workspace focused on understanding and implementing ROS 2 services (`srv`). It includes examples of defining and using custom service types, as well as client-server communication.

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
