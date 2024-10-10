# Implementing and Testing Custom ROS 2 Messages for Simulating Temperature Data

This document provides a complete and detailed guide for creating a ROS 2 project that simulates temperature data using custom messages, a Talker node to publish this data, and a Listener node to receive and display it. The documentation also highlights common mistakes, how to troubleshoot them, and the correct steps to follow.

## Overview

The goal of this project is to:

1. Create a **custom message** (`TemperatureMsg`) to handle temperature data.
2. Implement a **Talker node** that simulates temperature values and publishes them using the custom message.
3. Implement a **Listener node** that subscribes to the temperature topic and displays the received data.
4. **Build**, **test**, and **debug** the ROS 2 nodes and message interfaces using ROS 2 command-line tools.

---

## Table of Contents

1. [Setup the ROS 2 Workspace](#1-setup-the-ros-2-workspace)
2. [Create the Custom Message](#2-create-the-custom-message)
3. [Implement the Talker and Listener Nodes](#3-implement-the-talker-and-listener-nodes)
4. [Build the Workspace](#4-build-the-workspace)
5. [Test the Nodes and Custom Message via CLI](#5-test-the-nodes-and-custom-message-via-cli)
6. [Troubleshooting Common Errors](#6-troubleshooting-common-errors)
7. [Conclusion](#7-conclusion)

---

## 1. Setup the ROS 2 Workspace

### Correct Steps

1. **Create a new workspace** to hold your ROS 2 packages.

    ```bash
    mkdir -p ~/ros2_temperature_ws/src
    cd ~/ros2_temperature_ws
    ```

2. **Build the workspace** to initialize the build process:

    ```bash
    colcon build
    source install/setup.bash
    ```

### Common Mistakes

- **Skipping the workspace setup:** Without setting up the workspace correctly, the packages won't be able to build, and sourcing will fail. Ensure that every command is run in the correct workspace.

---

## 2. Create the Custom Message

We will now define a custom message (`TemperatureMsg`) to hold the temperature value, which will be a `float64` field.

### 2.1 Create the Custom Message Package

1. **Create the `custom_msgs` package**:

    ```bash
    cd ~/ros2_temperature_ws/src
    ros2 pkg create custom_msgs --build-type ament_cmake
    ```

2. **Navigate to the package directory** and create a folder for the message files:

    ```bash
    cd ~/ros2_temperature_ws/src/custom_msgs
    mkdir msg
    ```

3. **Create the `TemperatureMsg.msg` file**:

    ```bash
    echo "float64 temperature" > msg/TemperatureMsg.msg
    ```

### 2.2 Configure `CMakeLists.txt` and `package.xml`

1. **Edit the `CMakeLists.txt`**:
    Ensure the following lines are present for message generation:

    ```cmake
    find_package(ament_cmake REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/TemperatureMsg.msg"
    )

    ament_package()
    ```

2. **Edit `package.xml`** to declare the required dependencies for message generation:

    ```xml
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```

### Common Mistakes

- **Forgetting to include `rosidl_default_generators` in `CMakeLists.txt`:** This results in message generation failure during the build process.
- **Not declaring `rosidl_interface_packages` in `package.xml`:** This can lead to issues where ROS 2 tools don't recognize the message package as a valid interface package.

---

## 3. Implement the Talker and Listener Nodes

### 3.1 Create the Package for Nodes

1. **Create the `temperature_publisher` package** for the nodes:

    ```bash
    cd ~/ros2_temperature_ws/src
    ros2 pkg create temperature_publisher --build-type ament_python --dependencies rclpy custom_msgs
    ```

### 3.2 Implement the Talker Node

The Talker node will publish simulated temperature values using the `TemperatureMsg` custom message.

1. **Create the `talker.py` file** inside the package:

    ```bash
    cd ~/ros2_temperature_ws/src/temperature_publisher/temperature_publisher
    touch talker.py
    ```

2. **Edit `talker.py` to simulate temperature data**:

    ```python
    import rclpy
    from rclpy.node import Node
    import random
    from custom_msgs.msg import TemperatureMsg

    class TalkerNode(Node):
        def __init__(self):
            super().__init__('talker')
            self.publisher_ = self.create_publisher(TemperatureMsg, 'temperature', 10)
            self.timer = self.create_timer(1.0, self.publish_temperature)

        def publish_temperature(self):
            msg = TemperatureMsg()
            msg.temperature = random.uniform(15.0, 25.0)  # Simulated temperature
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing simulated temperature: {msg.temperature:.2f}°C')

    def main(args=None):
        rclpy.init(args=args)
        talker = TalkerNode()
        rclpy.spin(talker)
        talker.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### 3.3 Implement the Listener Node

1. **Create the `listener.py` file** inside the same package:

    ```bash
    touch listener.py
    ```

2. **Edit `listener.py` to receive the temperature data**:

    ```python
    import rclpy
    from rclpy.node import Node
    from custom_msgs.msg import TemperatureMsg

    class ListenerNode(Node):
        def __init__(self):
            super().__init__('listener')
            self.subscription = self.create_subscription(
                TemperatureMsg,
                'temperature',
                self.listener_callback,
                10
            )

        def listener_callback(self, msg):
            self.get_logger().info(f'Received temperature: {msg.temperature:.2f}°C')

    def main(args=None):
        rclpy.init(args=args)
        listener = ListenerNode()
        rclpy.spin(listener)
        listener.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### 3.4 Update `setup.py` for Entry Points

Edit the `setup.py` in `temperature_publisher` to register the entry points for the Talker and Listener nodes.

```python
from setuptools import setup

package_name = 'temperature_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Temperature Publisher for ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = temperature_publisher.talker:main',
            'listener = temperature_publisher.listener:main',
        ],
    },
)
```

### Common Mistakes

- **Forgetting to register entry points in `setup.py`:** This will prevent the nodes from being executed with `ros2 run`.
- **Not ensuring correct indentation or Python syntax in `setup.py`:** This can cause build errors.

---

## 4. Build the Workspace

Now that the message and nodes are implemented, we need to build the workspace.

1. **Build the workspace**:

    ```bash
    cd ~/ros2_temperature_ws
    colcon build
    ```

2. **Source the workspace**:

    ```bash
    source install/setup.bash
    ```

### Common Mistakes

- **Not sourcing the workspace after building:** If you forget to source the workspace, ROS 2 will not recognize the new packages and executables.

---

## 5. Test the Nodes and Custom Message via CLI

### 5.1 Test the Custom Message

1. **List available message types**:

    ```bash
    ros2 interface list | grep TemperatureMsg
    ```

2. **Display the structure of the custom message**:

    ```bash
    ros2 interface show custom_msgs/msg/TemperatureMsg
    ```

### 5.2 Test the Nodes

1. **Run the Talker node**:

    ```bash
    ros2 run temperature_publisher talker
    ```

2. **Run the Listener node** in a separate terminal:

    ```bash
    ros2 run temperature_publisher listener
    ```

3. **Check the published data**:

    ```bash
    ros2 topic echo /temperature
    ```

### Common Mistakes

- **Forgetting to run the nodes in separate terminals:** Both nodes should be run in different terminals for correct operation.
- **Not building or sourcing the workspace correctly:** If the workspace is not built or sourced, ROS 2 won’t recognize the new packages.

---

## 6. Troubleshooting Common Errors

### Error: `The message type 'custom_msgs/msg/TemperatureMsg' is invalid`

This error occurs when the message isn't properly generated. To resolve this:

1. **Ensure the message package

 is built**:
    ```bash
    colcon build --packages-select custom_msgs
    ```

2. **Check the message definition**:

    ```bash
    ros2 interface show custom_msgs/msg/TemperatureMsg
    ```

3. **Verify the `CMakeLists.txt` and `package.xml`** for correct configurations.

### Error: `Package 'temperature_publisher' not found`

This occurs when the package isn't properly installed. Ensure that:

1. **The `setup.py` is correctly configured** with entry points.
2. **The package is built and sourced** correctly.

---

## 7. Conclusion

In this comprehensive guide, we covered:

- Setting up a ROS 2 workspace.
- Creating a custom message for temperature data.
- Implementing Talker and Listener nodes using the custom message.
- Building the workspace and testing the functionality.
- Troubleshooting common errors and ensuring the system works as expected.

By following this guide, you will have a working ROS 2 project that simulates and transmits temperature data using custom messages, and you will be equipped to troubleshoot common issues that may arise.
