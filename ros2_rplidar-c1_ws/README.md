
# SLAMTEC LIDAR ROS2 Package for RPLIDAR C1

This ROS2 package provides a node for interfacing with the **SLAMTEC RPLIDAR C1** sensor. It allows seamless integration of the RPLIDAR C1 into your ROS2 (Humble) environment, supporting applications in mapping, navigation, and obstacle avoidance in robotic systems.

## Overview

The **RPLIDAR C1** sensor from SLAMTEC is a high-performance 360-degree 2D LiDAR for robotic applications. This ROS2 package includes everything you need to interface with the RPLIDAR C1, including:

- Launch files to start the LiDAR and view its output in RViz.
- Scripts to handle udev rules and set up the device.
- ROS nodes to publish sensor data for robotic applications.

## Prerequisites

To use this project, you will need:

- **ROS 2 Humble** installed on your machine.
- A **RPLIDAR C1** sensor.
- Proper permissions set up for accessing the USB port (see below).
  
### ROS2 Installation

Follow the installation guide for ROS2 Humble:

- [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html)

## Installation

### Setting up the Workspace

1. **Create the ROS2 Workspace**:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone the rplidar_ros package**:

   ```bash
   git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
   ```

### Compiling and Installing the Package

1. **Build the workspace**:

   ```bash
   cd ~/ros2_ws/
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```

2. **Set up the package environment**:

   ```bash
   source ./install/setup.bash
   ```

3. **Add environment setup to your `.bashrc`** for automatic sourcing:

   ```bash
   echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

4. **Create udev rules** for RPLIDAR C1:
   - Give read/write permissions to `/dev/ttyUSB0`:

     ```bash
     sudo chmod 777 /dev/ttyUSB0
     ```

   - Alternatively, run the udev rule script:

     ```bash
     cd src/rplidar_ros/
     ./scripts/create_udev_rules.sh
     ```

## Running the RPLIDAR Node

### Visualizing Lidar Data in RViz

1. **Launch the RPLIDAR C1 node and RViz**:

   ```bash
   ros2 launch sllidar_ros2 view_sllidar_c1_launch.py
   ```

2. **RViz Configuration**: The RViz configuration provided in `rviz/sllidar_ros2.rviz` allows you to visualize the Lidar data as it is being published.

### Troubleshooting

- **RViz crashing or not starting**: Ensure that all necessary dependencies for `rviz2` are installed.

  ```bash
  sudo apt install ros-humble-rviz2
  ```

- **Permission denied errors**: Make sure you have set the correct permissions for `/dev/ttyUSB0` using the provided udev rules script.

