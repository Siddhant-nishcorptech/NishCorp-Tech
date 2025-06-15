#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios
import time
import select

class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_controller')

        # Wheel speeds and joint positions
        self.wheel_speeds = [0.0, 0.0, 0.0, 0.0]  # wheel1, wheel2, wheel3, wheel4
        self.joint_states = {
            'rest': [0.0, 0.0, 0.0, 0.0, 0.0],  # Neutral position
            'standing': [-1.2, 0.8, 0.8, -1.0, 1.0]  # Standing position
        }
        self.current_joint_state = 'rest'

        # Publishers
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_commands', 10)
        self.joint_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)

        # Control parameters
        self.linear_speed = 2.0  # Speed for forward/backward
        self.turn_speed = 1.0  # Speed for turning
        self.prev_joint_keys = set()  # Track previous O/P key states

        # Save terminal settings
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)  # Enable non-blocking input

        # Control loop timer (20 Hz)
        self.create_timer(0.05, self.control_loop)

        # Publish initial joint state
        self.publish_joint_state()

        self.get_logger().info('Teleop Controller initialized. Use WASD for wheels, O/P to toggle joint states.')

    def get_key(self):
        """Read a single key from stdin non-blocking"""
        try:
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1).lower()
            return None
        except:
            return None

    def control_loop(self):
        """Process keyboard inputs and publish commands"""
        # Initialize wheel speeds (default to stop)
        temp_speeds = [0.0, 0.0, 0.0, 0.0]

        # Handle wheel control (WASD)
        key = self.get_key()
        if key == 'w':
            temp_speeds = [self.linear_speed, self.linear_speed, self.linear_speed, self.linear_speed]  # Forward
        elif key == 's':
            temp_speeds = [-self.linear_speed, -self.linear_speed, -self.linear_speed, -self.linear_speed]  # Backward
        elif key == 'a':
            temp_speeds = [-self.linear_speed, self.linear_speed, -self.linear_speed, self.linear_speed]  # Left turn
        elif key == 'd':
            temp_speeds = [self.linear_speed, -self.turn_speed, self.linear_speed, -self.linear_speed]  # Right turn

        # Invert wheel1 speed before publishing
        wheel_speeds = [-temp_speeds[0], temp_speeds[1], temp_speeds[2], temp_speeds[3]]

        # Publish wheel commands if changed
        if wheel_speeds != self.wheel_speeds:
            self.wheel_speeds = wheel_speeds
            wheel_msg = Float64MultiArray()
            wheel_msg.data = self.wheel_speeds
            self.wheel_pub.publish(wheel_msg)
            self.get_logger().info(f'Wheel commands: {self.wheel_speeds}')

        # Handle joint state toggle (O/P)
        current_joint_keys = set()
        if key == 'o':
            current_joint_keys.add('o')
        elif key == 'p':
            current_joint_keys.add('p')

        # Detect key press events (edge detection) for joint state toggle
        if 'o' in current_joint_keys and 'o' not in self.prev_joint_keys:
            self.current_joint_state = 'rest'
            self.publish_joint_state()
        elif 'p' in current_joint_keys and 'p' not in self.prev_joint_keys:
            self.current_joint_state = 'standing'
            self.publish_joint_state()

        self.prev_joint_keys = current_joint_keys

    def publish_joint_state(self):
        """Publish current joint state"""
        joint_msg = Float64MultiArray()
        joint_msg.data = self.joint_states[self.current_joint_state]
        self.joint_pub.publish(joint_msg)
        self.get_logger().info(f'Joint state: {self.current_joint_state}, Positions: {self.joint_states[self.current_joint_state]}')

    def __del__(self):
        """Restore terminal settings on exit"""
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

def main(args=None):
    rclpy.init(args=args)
    controller = TeleopController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop wheels on exit
        wheel_msg = Float64MultiArray()
        wheel_msg.data = [0.0, 0.0, 0.0, 0.0]
        controller.wheel_pub.publish(wheel_msg)
        controller.__del__()  # Restore terminal settings
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
