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
        # Convert the list of bytes back into a single bytes object
        serialized_data = b''.join(msg.data)
        # Deserialize the data using pickle
        received_data = pickle.loads(serialized_data)
        self.get_logger().info(f'Received serialized message: {received_data}')

def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
