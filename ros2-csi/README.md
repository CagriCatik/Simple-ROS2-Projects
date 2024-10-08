# CSI Camera ROS 2 Node

This ROS 2 package streams video from a CSI camera (e.g., Raspberry Pi camera) connected to an NVIDIA Jetson device. The video frames are captured using OpenCV with a GStreamer pipeline and published as `sensor_msgs/msg/Image` messages to a ROS 2 topic. The node can be launched using a provided launch file.

## Features

- Stream video from a CSI camera using OpenCV with GStreamer.
- Publish the video stream as `sensor_msgs/Image` messages.
- Easily configurable resolution, framerate, and flip settings via the GStreamer pipeline.
- Includes a launch file for starting the node.

## Requirements

- **ROS 2** (tested with Foxy, Galactic, or Humble).
- **Python 3.x**
- **OpenCV with GStreamer support** (for accessing the CSI camera).
- **cv_bridge** (for converting OpenCV images to ROS image messages).

### Install Dependencies

Make sure you have OpenCV and `cv_bridge` installed. You can install them using the following commands:

```bash
# Install OpenCV with GStreamer support
sudo apt-get install python3-opencv

# Install ROS 2 cv_bridge package
sudo apt-get install ros-<ros-distro>-cv-bridge
```

Replace `<ros-distro>` with your ROS 2 distribution (`foxy`, `galactic`, `humble`, etc.).

## Installation

1. **Create a ROS 2 workspace (if you don't have one already)** and clone the package:

   ```bash
   mkdir -p ~/csi_ros2/src
   cd ~/csi_ros2/src
   git clone https://github.com/cagricatik/csi_camera.git
   ```

2. **Install Python dependencies**:

   ```bash
   pip install -r ~/csi_ros2/src/csi_camera/requirements.txt
   ```

3. **Build the package**:

   Navigate to the root of your workspace and build the package:

   ```bash
   cd ~/csi_ros2
   colcon build --packages-select csi_camera
   ```

4. **Source the workspace**:

   After building the package, ensure that you source the workspace to overlay the newly built packages:

   ```bash
   source install/setup.bash
   ```

## Running the Node

You can run the CSI camera node using the provided launch file.

### Using the Launch File

Run the following command to start the node:

```bash
ros2 launch csi_camera csi_camera_launch.py
```

This will:

- Start the CSI camera stream.
- Publish the video frames as `sensor_msgs/Image` messages to the `/csi_camera/image_raw` topic.

### Directly Running the Node

Alternatively, you can directly run the node using:

```bash
ros2 run csi_camera csi_camera_node
```

## Viewing the Video Stream

To view the video stream, you can use ROS 2 tools like `rqt_image_view` to subscribe to the image topic:

```bash
ros2 run rqt_image_view rqt_image_view
```

In `rqt_image_view`, select the `/csi_camera/image_raw` topic to view the live stream.

## Customization

You can adjust the GStreamer pipeline settings inside the `csi_camera_node.py` script. Here are some configurable parameters:

- **Resolution**: `capture_width`, `capture_height`
- **Display Resolution**: `display_width`, `display_height`
- **Framerate**: `framerate`
- **Flip Method**: `flip_method` (0 = no flip, 1 = horizontal flip, 2 = vertical flip, etc.)

Example snippet to adjust the GStreamer pipeline in the script:

```python
def gstreamer_pipeline(self, capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=30, flip_method=0):
    return (
        f"nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int){capture_width}, height=(int){capture_height}, "
        f"format=(string)NV12, framerate=(fraction){framerate}/1 ! nvvidconv flip-method={flip_method} ! "
        f"video/x-raw, width=(int){display_width}, height=(int){display_height}, format=(string)BGRx ! "
        "videoconvert ! video/x-raw, format=(string)BGR ! appsink"
    )
```

## Project Structure

The directory structure of this ROS 2 package is as follows:

```sh
csi_camera/
├── launch/
│   └── csi_camera_launch.py       # Launch file to run the node
├── csi_camera/
│   └── csi_camera_node.py         # The Python node
├── setup.py                       # Python setup file
├── package.xml                    # ROS 2 package metadata
├── requirements.txt               # Python dependencies
├── resource/
│   └── csi_camera                 # Empty file required by ROS 2 for Python packages
```
