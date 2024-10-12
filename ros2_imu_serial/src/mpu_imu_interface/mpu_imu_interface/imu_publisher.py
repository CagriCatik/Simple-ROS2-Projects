import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        
        # Create a publisher for the IMU data
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        
        # Open the serial port to which Arduino is connected (adjust the port as needed)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 19200, timeout=1)
        
        # Create a timer to call the function every 0.1 second (10 Hz)
        self.timer = self.create_timer(0.01, self.publish_imu_data)

    def quaternion_to_euler(imu_msg: Imu):
        q = imu_msg.orientation
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def publish_imu_data(self):
        try:
            # Read the data from the serial port
            line = self.serial_port.readline().decode('utf-8').strip()

            # Assuming the data format is:
            # Roll: <value>, Pitch: <value>, Yaw: <value>
            if line:
                # Parse the roll, pitch, and yaw values
                data = line.split(',')
                roll = float(data[0].split(": ")[1])
                pitch = float(data[1].split(": ")[1])
                yaw = float(data[2].split(": ")[1])

                # Create and populate an Imu message
                imu_msg = Imu()
                
                # In this case, we only publish orientation angles (roll, pitch, yaw)
                # Note: IMU message requires orientation to be in quaternion format
                imu_msg.orientation.x = roll
                imu_msg.orientation.y = pitch
                imu_msg.orientation.z = yaw
                imu_msg.orientation.w = 1.0  # You can compute the real w based on quaternion conversion

                # Publish the IMU data
                self.publisher_.publish(imu_msg)
                self.get_logger().info(f'Published IMU data - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}')

        except Exception as e:
            self.get_logger().error(f'Error reading from serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
