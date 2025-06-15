#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Initialize joint positions
        self.joint_positions = [0.0] * 5  # For j1, j2, j3, j4, j5

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)

        # Subscriber for joint commands
        self.create_subscription(Float64MultiArray, '/joint_commands', self.joint_command_callback, 10)

        # Control loop timer (20 Hz)
        self.create_timer(0.05, self.publish_positions)

        self.get_logger().info('Joint Controller initialized')

    def joint_command_callback(self, msg):
        """Store joint positions"""
        if len(msg.data) == 5:
            self.joint_positions = list(msg.data)  # Store j1, j2, j3, j4, j5 positions
            self.get_logger().info(f'Received: {msg.data}, Stored: {self.joint_positions}')
        else:
            self.get_logger().warn(f'Invalid command length: {len(msg.data)}')

    def publish_positions(self):
        """Publish stored joint positions"""
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = JointController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
