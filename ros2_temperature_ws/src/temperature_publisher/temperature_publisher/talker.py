import rclpy
from rclpy.node import Node
import random
from custom_msgs.msg import TemperatureMsg  # Import the custom message

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(TemperatureMsg, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)

    def publish_temperature(self):
        msg = TemperatureMsg()
        msg.temperature = random.uniform(15.0, 25.0)  # Simulate a temperature between 15°C and 25°C
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
