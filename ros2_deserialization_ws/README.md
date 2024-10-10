# Custom ROS 2 Messages Using Python Serialization

This documentation covers the entire process of creating, implementing, and testing a custom ROS 2 message type (`UltimateMsg`) with Python serialization and deserialization using the `pickle` library. The chat progress reflects troubleshooting steps, adjustments, and the solution to serialize and deserialize complex Python objects between ROS 2 nodes (Talker and Listener).

---

## Table of Contents

1. **Project Setup**
2. **Creating a Custom ROS 2 Message**
3. **Implementing Talker and Listener Nodes**
4. **Compiling and Building the Workspace**
5. **Testing the ROS 2 Nodes**
6. **Troubleshooting and Solutions**
7. **Verification via Command-Line Tools**

---

## 1. Project Setup

Before we dive into creating custom messages and nodes, it is essential to have a working ROS 2 environment and workspace. You can set up a new ROS 2 workspace if needed:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

This workspace, named `ros2_ws`, will hold all ROS 2 packages used in this project. You’ll need to create two ROS 2 packages:

- `custom_msgs`: For custom message definitions.
- `custom_nodes`: For the ROS 2 Talker and Listener nodes.

---

## 2. Creating a Custom ROS 2 Message

In ROS 2, custom messages allow you to send specialized data structures between nodes. For this project, the custom message is designed to serialize complex Python objects into a `byte[]` format.

### Steps

1. **Create the `custom_msgs` package:**

```bash
cd ~/ros2_ws/src
ros2 pkg create custom_msgs --build-type ament_cmake
```

2. **Add the message definition:**

Create a folder called `msg` inside the `custom_msgs` package and define the message file:

```bash
mkdir -p ~/ros2_ws/src/custom_msgs/msg
echo "byte[] data" > ~/ros2_ws/src/custom_msgs/msg/UltimateMsg.msg
```

3. **Update `CMakeLists.txt`:**

Add the necessary configuration to `CMakeLists.txt` for message generation:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UltimateMsg.msg"
)
ament_package()
```

4. **Update `package.xml`:**

Ensure the `package.xml` declares the appropriate dependencies and the required `rosidl_interface_packages` tag:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

This defines the custom message `UltimateMsg` with a single field, `data`, which is a list of bytes (`byte[]`).

---

## 3. Implementing Talker and Listener Nodes

Now we will implement two ROS 2 nodes (Talker and Listener) that will use the `UltimateMsg` to send serialized Python objects between nodes.

### Talker Node

The `talker.py` node will serialize a Python dictionary using `pickle`, convert it into a list of bytes, and publish it using the `UltimateMsg`.

```python
import rclpy
from rclpy.node import Node
import pickle
from custom_msgs.msg import UltimateMsg

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(UltimateMsg, 'custom_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = UltimateMsg()
        data_to_serialize = {'key': 'value', 'numbers': [1, 2, 3]}
        serialized_data = pickle.dumps(data_to_serialize)
        msg.data = [bytes([b]) for b in serialized_data]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing serialized message: {data_to_serialize}')

def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()
```

### Listener Node

The `listener.py` node subscribes to the `custom_topic`, receives the serialized message, and deserializes it using `pickle`.

```python
import rclpy
from rclpy.node import Node
import pickle
from custom_msgs.msg import UltimateMsg

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            UltimateMsg,
            'custom_topic',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        serialized_data = b''.join(msg.data)
        received_data = pickle.loads(serialized_data)
        self.get_logger().info(f'Received serialized message: {received_data}')

def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()
```

## 4. Compiling and Building the Workspace

After implementing both nodes and the custom message, you need to build the ROS 2 workspace to ensure everything is correctly set up.

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

This builds both the `custom_msgs` and `custom_nodes` packages, ensuring that the message and nodes are correctly registered.

---

## 5. Testing the ROS 2 Nodes

### Running the Talker Node

Start by running the talker node in one terminal:

```bash
ros2 run custom_nodes talker
```

You should see logs like:

```sh
[INFO] [talker]: Publishing serialized message: {'key': 'value', 'numbers': [1, 2, 3]}
```

### Running the Listener Node

In another terminal, run the listener node:

```bash
ros2 run custom_nodes listener
```

If everything is working correctly, you should see the deserialized Python object:

```sh
[INFO] [listener]: Received serialized message: {'key': 'value', 'numbers': [1, 2, 3]}
```

---

## 6. Troubleshooting and Solutions

Several issues were encountered and resolved during implementation. Here’s a summary of key problems and their solutions:

1. **Error: The 'data' field must be a set or sequence and each value of type 'bytes'**
   - **Solution**: Ensure that each element of the `data` field in the message is a byte object. This was achieved by converting the serialized data using `[bytes([b]) for b in serialized_data]`.

2. **Error: TypeError: a bytes-like object is required, not 'list'**
   - **Solution**: In the listener node, the list of bytes was converted back into a single `bytes` object using `b''.join(msg.data)` before passing it to `pickle.loads`.

---

## 7. Verification via Command-Line Tools

You can use the ROS 2 CLI tools to verify the message interface and data exchange.

1. **List Available Message Types:**

   Verify that the custom message `UltimateMsg` is registered:

   ```bash
   ros2 interface list | grep UltimateMsg
   ```

   You should see:

   ```sh
   custom_msgs/msg/UltimateMsg
   ```

2. **Inspect the Topic:**

   To check if the talker node is publishing data on the topic, use:

   ```bash
   ros2 topic echo /custom_topic
   ```

   You’ll see the raw byte data being published by the talker node.

3. **Show Message Definition:**

   Check the structure of the custom message with:

   ```bash
   ros2 interface show custom_msgs/msg/UltimateMsg
   ```

   The expected output:

   ```sh
   byte[] data
   ```

---

## Conclusion

In this project, you learned how to:

- Create custom ROS 2 messages (`UltimateMsg`) to serialize and transmit complex Python objects using `pickle`.
- Implement `talker` and `listener` nodes to serialize and deserialize these objects.
- Build and test the ROS 2 workspace, troubleshoot common issues, and verify the system using ROS 2 CLI tools.

This approach can be extended to other applications where transmitting complex Python objects between ROS 2 nodes is required.
