import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import subprocess
import requests

class InitialPoseListener(Node):

    def __init__(self):
        super().__init__('initial_pose_listener')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.listener_callback,
            10)
        self.initialized = False  # Flag to ensure actions are only taken once
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if not self.initialized:
            self.get_logger().info('Received initial pose for the first time, activating keepout filter...')
            self.activate_keepout_filter()
            self.initialized = True
        else:
            self.get_logger().info('Initial pose received again, no action needed.')

    def activate_keepout_filter(self):
        # Run lifecycle command to configure and activate the filter mask
        self.run_lifecycle_command('/filter_mask_server', 'configure')
        self.run_lifecycle_command('/filter_mask_server', 'activate')
        # Check if the costmap_filter_info_server node exists before configuring it
        if self.check_node_active('/costmap_filter_info_server'):
            self.run_lifecycle_command('/costmap_filter_info_server', 'configure')
            self.run_lifecycle_command('/costmap_filter_info_server', 'activate')
        # Send a request to refresh the map on the backend after successful activation
        self.refresh_map()

    def run_lifecycle_command(self, node_name, transition):
        command = ['ros2', 'lifecycle', 'set', node_name, transition]
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f'Failed to {transition} {node_name}: {result.stderr}')
        else:
            self.get_logger().info(f'Successfully {transition}d {node_name}')

    def check_node_active(self, node_name):
        command = ['ros2', 'node', 'info', node_name]
        result = subprocess.run(command, capture_output=True, text=True)
        return result.returncode == 0

    def refresh_map(self):
        try:
            self.get_logger().info('Requesting map refresh from backend...')
            response = requests.post("http://localhost:5258/maps/refresh")
            if response.status_code == 200:
                self.get_logger().info('Successfully requested map refresh.')
            else:
                self.get_logger().error(f'Failed to request map refresh. Status code: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Exception while requesting map refresh: {e}')

def main(args=None):
    rclpy.init(args=args)

    initial_pose_listener = InitialPoseListener()

    rclpy.spin(initial_pose_listener)

    initial_pose_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
