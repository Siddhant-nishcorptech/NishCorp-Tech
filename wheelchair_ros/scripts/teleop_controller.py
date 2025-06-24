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
            'standing': [-1.2, 0.6, 0.6, -1.2, 1.2]  # Standing position
        }
        self.current_joint_state = 'rest'
        self.target_joint_state = 'rest'
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_speed = 1.0  # Speed of joint movement (radians per second)
        self.last_joint_update = time.time()

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

        # Control loop timer (50 Hz for smoother transitions)
        self.create_timer(0.02, self.control_loop)

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
        # Get key input
        key = self.get_key()

        # Only update wheel speeds if a key is pressed
        if key is not None:
            temp_speeds = self.wheel_speeds.copy()  # Maintain previous speeds by default

            if key == 'w':
                temp_speeds = [self.linear_speed, self.linear_speed, self.linear_speed, self.linear_speed]  # Forward
            elif key == 's':
                temp_speeds = [-self.linear_speed, -self.linear_speed, -self.linear_speed, -self.linear_speed]  # Backward
            elif key == 'a':
                temp_speeds = [-self.linear_speed, self.linear_speed, -self.linear_speed, self.linear_speed]  # Left turn
            elif key == 'd':
                temp_speeds = [self.linear_speed, -self.turn_speed, self.linear_speed, -self.linear_speed]  # Right turn
            elif key == ' ':  # Space to stop
                temp_speeds = [0.0, 0.0, 0.0, 0.0]

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
            if key == 'o':
                self.target_joint_state = 'rest'
                self.joint_speed = 2.0  # Increase speed for downward movement
            elif key == 'p':
                self.target_joint_state = 'standing'
                self.joint_speed = 1.0  # Normal speed for upward movement

        # Update and publish joint positions
        self.update_joint_positions()
        self.publish_joint_state()

    def publish_joint_state(self):
        """Publish current joint state"""
        joint_msg = Float64MultiArray()
        joint_msg.data = self.current_joint_positions
        self.joint_pub.publish(joint_msg)
        self.get_logger().debug(f'Joint positions: {self.current_joint_positions}')

    def update_joint_positions(self):
        """Update joint positions with smooth interpolation"""
        current_time = time.time()
        dt = current_time - self.last_joint_update
        self.last_joint_update = current_time

        target_positions = self.joint_states[self.target_joint_state]
        max_step = self.joint_speed * dt

        # Calculate new positions with smooth interpolation
        for i in range(len(self.current_joint_positions)):
            diff = target_positions[i] - self.current_joint_positions[i]
            if abs(diff) < max_step:
                self.current_joint_positions[i] = target_positions[i]
            else:
                step = max_step if diff > 0 else -max_step
                self.current_joint_positions[i] += step

        # Check if we've reached the target state
        if all(abs(self.current_joint_positions[i] - target_positions[i]) < 0.01 for i in range(len(self.current_joint_positions))):
            self.current_joint_state = self.target_joint_state

    def __del__(self):
        """Restore terminal settings on exit"""
        if hasattr(self, 'fd') and hasattr(self, 'old_settings'):
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
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
