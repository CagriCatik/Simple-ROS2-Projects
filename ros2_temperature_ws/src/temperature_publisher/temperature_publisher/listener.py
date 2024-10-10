import rclpy
from rclpy.node import Node
from custom_msgs.msg import TemperatureMsg
import matplotlib.pyplot as plt
from collections import deque

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')

        # Create a subscription to the temperature topic
        self.subscription = self.create_subscription(
            TemperatureMsg,
            'temperature',
            self.listener_callback,
            10
        )

        # Deque to store data points for real-time plotting
        self.temperatures = deque(maxlen=100)  # Store the last 100 temperature values
        self.fig, self.ax = plt.subplots()

        # Set up the plot
        self.ax.set_title('Real-Time Temperature Plot')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Temperature (°C)')
        self.line, = self.ax.plot([], [], lw=2)  # Empty line that will be updated

        # Start the plot
        plt.ion()
        plt.show()

    def listener_callback(self, msg):
        # Add the new temperature value to the deque
        self.temperatures.append(msg.temperature)
        self.get_logger().info(f'Received temperature: {msg.temperature:.2f}°C')

        # Update the plot
        self.update_plot()

    def update_plot(self):
        # Update the plot with the latest temperature data
        self.line.set_data(range(len(self.temperatures)), self.temperatures)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    listener = ListenerNode()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
