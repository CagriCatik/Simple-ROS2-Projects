import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoCommandSubscriber(Node):
    def __init__(self):
        super().__init__('arduino_command_subscriber')
        self.subscription = self.create_subscription(
            String,
            'led_command',
            self.listener_callback,
            10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust the port
        self.command_to_send = '0'  # Default command (turn off)
    
    def listener_callback(self, msg):
        # Receive command from the topic and set it as the command to send
        self.command_to_send = msg.data
        self.get_logger().info(f'Received command: {self.command_to_send}')
        self.send_command()

    def send_command(self):
        try:
            # Send command to Arduino via Serial
            self.serial_port.write(self.command_to_send.encode())
            self.get_logger().info(f'Sent command: {self.command_to_send}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial exception: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommandSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.serial_port.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
