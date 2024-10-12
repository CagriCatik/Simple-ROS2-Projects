# **ROS 2 IMU Interface Project**

This project demonstrates how to interface an **MPU6050** IMU sensor with **ROS 2** using an **Arduino** to collect and transmit sensor data (via serial communication) and a ROS 2 node to receive, process, and publish this data in real time.

The project includes:

- An **Arduino script** to gather IMU data (roll, pitch, yaw) and send it via serial.
- A **ROS 2 node** written in Python that subscribes to the serial port, receives the data, and publishes it as a `sensor_msgs/Imu` message.
- Tools for **visualizing and plotting** the IMU data using **PlotJuggler**.

## **Project Structure**

```sh
ros2_imu_serial/
├── arduino/
│   ├── imu_serial/
│   │   └── imu_serial.ino            # Arduino sketch for collecting MPU6050 data
├── install-arduino-ide.sh            # Script to install Arduino IDE
├── install-plotjuggler.sh            # Script to install PlotJuggler
├── requirements.txt                  # Python dependencies for ROS 2 node
└── src/
    └── mpu_imu_interface/            # ROS 2 package
        ├── mpu_imu_interface/
        │   ├── imu_publisher.py      # ROS 2 node to publish IMU data
        ├── package.xml               # ROS 2 package description
        ├── setup.cfg
        └── setup.py                  # Setup file for ROS 2 Python package
```

## **Setup Instructions**

### **Prerequisites**

- **Ubuntu 22.04** (or compatible with ROS 2 Humble).
- **ROS 2 Humble** distribution installed. Follow the [ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation.html) if ROS 2 is not installed.
- **Arduino IDE** installed to flash the Arduino with the IMU data collection code.

#### **Python Dependencies**

You can install the Python dependencies using `requirements.txt`:

```bash
pip install -r requirements.txt
```

Make sure to include `pyserial`, `smbus2`, and other dependencies for reading IMU data via I2C.

### **Step 1: Flash the Arduino**

The Arduino code reads the data from the MPU6050 sensor over I2C and sends it to the ROS 2 node via serial communication. To upload the Arduino sketch:

1. Open the **Arduino IDE**.
2. Load the `arduino/imu_serial/imu_serial.ino` sketch.
3. Connect your Arduino to your computer and upload the sketch.
4. Ensure the **MPU6050** is connected to the Arduino via the I2C interface (typically SDA on A4 and SCL on A5 for an Arduino Uno).

#### **MPU6050 Connection**

- **SDA** → A4 (on Arduino Uno)
- **SCL** → A5 (on Arduino Uno)
- **VCC** → 5V (or 3.3V, depending on your board)
- **GND** → GND

The Arduino will output serial data containing the roll, pitch, and yaw values, which will be read by the ROS 2 node.

### **Step 2: Build the ROS 2 Package**

1. Clone the repository into your ROS 2 workspace (e.g., `~/ros2_ws/src/`).
2. Navigate to your workspace:

   ```bash
   cd ~/ros2_imu_serial/
   ```

3. Build the package:

   ```bash
   colcon build
   ```

4. Source the setup file:

   ```bash
   source install/setup.bash
   ```

### **Step 3: Run the ROS 2 Node**

To run the ROS 2 node that reads serial data from the Arduino and publishes it as `sensor_msgs/Imu`:

1. Ensure the Arduino is connected to `/dev/ttyUSB0` (or the correct port).
2. Run the ROS 2 node:

   ```bash
   ros2 run mpu_imu_interface imu_publisher
   ```

This node will continuously publish IMU data (roll, pitch, yaw) to the `/imu/data` topic.

### **Step 4: Visualize IMU Data Using PlotJuggler**

#### Install PlotJuggler

There is a script provided to install PlotJuggler automatically. Run the following command to install PlotJuggler:

```bash
sudo bash install-plotjuggler.sh
```

This script will install PlotJuggler and the necessary plugins for ROS 2.

#### Using PlotJuggler to Visualize IMU Data

1. **Launch PlotJuggler**:

   ```bash
   plotjuggler
   ```

2. **Add the ROS 2 plugin**:

   - In the PlotJuggler GUI, click on **File > Data Source > ROS 2**.
   - Click **Start** to begin streaming ROS 2 data.
   - You should now see a list of available topics.

3. **Plot the IMU Data**:

   - Select the `/imu/data` topic (or whichever topic you are publishing the IMU data to).
   - Expand the topic to select the specific fields you want to plot, such as:
     - **Orientation (quaternion)**:

       ```sh
       /imu/data/orientation/x
       /imu/data/orientation/y
       /imu/data/orientation/z
       /imu/data/orientation/w
       ```

     - **Angular velocity**:

       ```sh
       /imu/data/angular_velocity/x
       /imu/data/angular_velocity/y
       /imu/data/angular_velocity/z
       ```

     - **Linear acceleration**:

       ```sh
       /imu/data/linear_acceleration/x
       /imu/data/linear_acceleration/y
       /imu/data/linear_acceleration/z
       ```

4. **Adjust the Time Window**: In PlotJuggler, you can adjust the time window, zoom, and pan through the data as needed.

### **Project Components**

#### 1. **Arduino Code (`arduino/imu_serial/imu_serial.ino`)**

The Arduino sketch reads data from the MPU6050 using the I2C interface, calculates roll, pitch, and yaw, and sends this data over serial to the ROS 2 node.

#### 2. **ROS 2 Node (`src/mpu_imu_interface/imu_publisher.py`)**

The `imu_publisher.py` node reads data from the serial port, parses it, and publishes it as a `sensor_msgs/Imu` message to the `/imu/data` topic.

#### 3. **PlotJuggler**

**PlotJuggler** is used to visualize and plot the IMU data in real time. It allows for advanced data analysis and customization of plots.

### **Future Improvements**

- Implement more robust error handling for the serial communication.
- Add support for additional IMU sensors.
- Implement data fusion techniques to combine accelerometer and gyroscope data more accurately.

### **Troubleshooting**

- **No data in PlotJuggler**: Ensure the ROS 2 node is correctly publishing to `/imu/data`. Use `ros2 topic echo /imu/data` to verify.
- **Serial port issues**: Verify the Arduino is connected to the correct serial port (`/dev/ttyUSB0` or another port) and that you have permission to access it (`sudo usermod -aG dialout $USER`).
