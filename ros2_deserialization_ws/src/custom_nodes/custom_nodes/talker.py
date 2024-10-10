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
        # Create a Python object to serialize
        data_to_serialize = {'key': 'value', 'numbers': [1, 2, 3]}
        # Serialize the Python object
        serialized_data = pickle.dumps(data_to_serialize)
        # Convert serialized_data to a list of bytes (each element being a byte object)
        msg.data = [bytes([b]) for b in serialized_data]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing serialized message: {data_to_serialize}')

def main(args=None):
    rclpy.init(args=args)
    talker = TalkerNode()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
