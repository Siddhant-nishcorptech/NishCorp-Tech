#!/usr/bin/env python3
'''
Subscribes from /wheel_commands and then publish the motor speeds to the appropriate motor controllers in gazebo.
'''

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Initialize wheel speeds
        self.wheel_speeds = [0.0] * 4

        # Publisher for wheel commands
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_controller/commands', 10)

        # Subscriber for wheel commands
        self.create_subscription(Float64MultiArray, '/wheel_commands', self.wheel_command_callback, 10)

        # Control loop timer (20 Hz)
        self.create_timer(0.05, self.publish_speeds)

        self.get_logger().info('Motor Controller initialized')

    def wheel_command_callback(self, msg):
        """Store wheel speeds, inverting wheel1"""
        if len(msg.data) == 4:
            self.wheel_speeds = [msg.data[0], msg.data[1], msg.data[2], msg.data[3]]
            self.get_logger().info(f'Received: {msg.data}, Stored: {self.wheel_speeds}')
        else:
            self.get_logger().warn(f'Invalid command length: {len(msg.data)}')

    def publish_speeds(self):
        """Publish stored wheel speeds"""
        msg = Float64MultiArray()
        msg.data = self.wheel_speeds
        self.wheel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = MotorController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
