import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        self.publisher_ = self.create_publisher(BatteryState, '/battery_state', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_level = 100.0

    def timer_callback(self):
        msg = BatteryState()
        msg.percentage = self.battery_level / 100.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Battery Level: {self.battery_level}%')
        self.battery_level -= 0.1  # Simulate battery drain

def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
