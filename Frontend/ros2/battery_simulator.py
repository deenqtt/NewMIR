import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist

class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        self.publisher_ = self.create_publisher(BatteryState, '/battery_state', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_level = 100.0
        self.is_moving = False
        self.cmd_vel_subscriber = None
        self.cmd_vel_checked = False

    def check_cmd_vel_topic(self):
        topics = self.get_topic_names_and_types()
        if any('/cmd_vel' in topic for topic, _ in topics):
            if self.cmd_vel_subscriber is None:
                self.cmd_vel_subscriber = self.create_subscription(
                    Twist,
                    '/cmd_vel',
                    self.velocity_callback,
                    10)
                self.get_logger().info('Subscribed to /cmd_vel topic.')
            return True
        return False

    def velocity_callback(self, msg):
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.is_moving = True
        else:
            self.is_moving = False

    def timer_callback(self):
        if not self.cmd_vel_checked:
            if self.check_cmd_vel_topic():
                self.cmd_vel_checked = True
                self.get_logger().info('cmd_vel topic found, starting battery simulation.')
            else:
                self.get_logger().info('Waiting for cmd_vel topic...')
                return  # Skip publishing battery state if cmd_vel topic not found

        # Adjust battery level based on whether the robot is moving or not
        if self.is_moving:
            self.battery_level -= 0.1  # Simulate slower battery drain while moving
        else:
            self.battery_level += 0.2  # Simulate slower battery charging if not moving

        # Clamp the battery level between 0 and 100
        if self.battery_level > 100.0:
            self.battery_level = 100.0
        elif self.battery_level < 0:
            self.battery_level = 0.0

        msg = BatteryState()
        msg.percentage = self.battery_level / 100.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Battery Level: {self.battery_level}%')

def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
