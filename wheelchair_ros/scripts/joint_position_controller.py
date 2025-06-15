#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math
import numpy as np

class WheelchairController(Node):
    def __init__(self):
        super().__init__('wheelchair_controller')

        # Publishers
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_controller/commands',
            10
        )

        # Subscriber for joint states (for feedback)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Current joint positions
        self.current_positions = [0.0] * 9
        self.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7', 'j8', 'j9']

        # Predefined positions and movements
        self.positions = {
            'rest': {
                'j1-j5': [0.0, 0.0, 0.0, 0.0, 0.0],  # All joints in neutral position
                'wheels': [0.0, 0.0, 0.0, 0.0]
            },
            'standing_prep': {
                'j1-j5': [-0.5, 0.3, 0.3, -0.5, 0.5],  # Gentle standing preparation
                'wheels': [0.0, 0.0, 0.0, 0.0]
            },
            'standing': {
                'j1-j5': [-1.2, 0.8, 0.8, -1.0, 1.0],  # Full standing position
                'wheels': [0.0, 0.0, 0.0, 0.0]
            },
            'forward_slow': {
                'j1-j5': [0.0, 0.0, 0.0, 0.0, 0.0],
                'wheels': [1.0, 1.0, 1.0, 1.0]  # Slow forward movement
            },
            'forward_fast': {
                'j1-j5': [0.0, 0.0, 0.0, 0.0, 0.0],
                'wheels': [3.0, 3.0, 3.0, 3.0]  # Fast forward movement
            },
            'turn_left': {
                'j1-j5': [0.0, 0.0, 0.0, 0.0, 0.0],
                'wheels': [-1.0, -1.0, 1.0, 1.0]  # Turn left
            },
            'turn_right': {
                'j1-j5': [0.0, 0.0, 0.0, 0.0, 0.0],
                'wheels': [1.0, 1.0, -1.0, -1.0]  # Turn right
            },
            'backward': {
                'j1-j5': [0.0, 0.0, 0.0, 0.0, 0.0],
                'wheels': [-2.0, -2.0, -2.0, -2.0]  # Backward movement
            }
        }

        # Control parameters
        self.current_target = 'rest'
        self.command_frequency = 20.0  # 20 Hz for smooth control
        self.position_tolerance = 0.05  # Tolerance for position control
        self.interpolation_steps = 50  # Steps for smooth motion

        # Interpolation variables
        self.interpolating = False
        self.start_positions = [0.0] * 5
        self.target_positions = [0.0] * 5
        self.interpolation_step = 0

        # Timers
        self.control_timer = self.create_timer(1.0/self.command_frequency, self.control_loop)

        # Demo sequence timer
        self.demo_timer = self.create_timer(1.0, self.demo_sequence)
        self.demo_step = 0
        self.demo_start_time = self.get_clock().now()

        self.get_logger().info('Wheelchair Controller initialized')
        self.get_logger().info('Starting demo sequence...')

    def joint_state_callback(self, msg):
        """Update current joint positions from joint states"""
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_positions[idx] = msg.position[i]

    def smooth_position_transition(self, target_name):
        """Start smooth transition to target position"""
        if target_name not in self.positions:
            self.get_logger().warn(f'Unknown target position: {target_name}')
            return

        self.start_positions = self.current_positions[:5].copy()  # j1-j5 only
        self.target_positions = self.positions[target_name]['j1-j5'].copy()
        self.interpolation_step = 0
        self.interpolating = True
        self.current_target = target_name

        self.get_logger().info(f'Starting smooth transition to: {target_name}')

    def get_interpolated_positions(self):
        """Get current interpolated positions"""
        if not self.interpolating:
            return self.positions[self.current_target]['j1-j5']

        # Use smooth interpolation (ease-in-out)
        progress = self.interpolation_step / self.interpolation_steps
        smooth_progress = 0.5 * (1.0 - math.cos(progress * math.pi))

        interpolated = []
        for start, target in zip(self.start_positions, self.target_positions):
            interpolated.append(start + (target - start) * smooth_progress)

        self.interpolation_step += 1
        if self.interpolation_step >= self.interpolation_steps:
            self.interpolating = False
            self.get_logger().info(f'Reached target position: {self.current_target}')

        return interpolated

    def control_loop(self):
        """Main control loop"""
        # Get current target positions
        current_pos = self.get_interpolated_positions()
        current_wheels = self.positions[self.current_target]['wheels']

        # Publish position commands with higher frequency for responsiveness
        position_msg = Float64MultiArray()
        position_msg.data = current_pos
        self.position_pub.publish(position_msg)

        # Publish wheel commands
        wheel_msg = Float64MultiArray()
        wheel_msg.data = current_wheels
        self.wheel_pub.publish(wheel_msg)

    def demo_sequence(self):
        """Demo sequence to test all movements"""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.demo_start_time).nanoseconds / 1e9

        # Define demo sequence with timing
        demo_sequence = [
            (0, 'rest', 'Starting in rest position'),
            (3, 'standing_prep', 'Preparing for standing'),
            (8, 'standing', 'Moving to standing position'),
            (13, 'rest', 'Returning to rest'),
            (18, 'forward_slow', 'Moving forward slowly'),
            (23, 'rest', 'Stopping'),
            (28, 'turn_left', 'Turning left'),
            (33, 'rest', 'Stopping turn'),
            (38, 'turn_right', 'Turning right'),
            (43, 'rest', 'Stopping turn'),
            (48, 'backward', 'Moving backward'),
            (53, 'rest', 'Final rest position'),
        ]

        for time_point, position, description in demo_sequence:
            if elapsed >= time_point and elapsed < time_point + 0.5:  # 0.5s window
                if self.current_target != position or not hasattr(self, 'last_demo_action'):
                    self.get_logger().info(f'Demo: {description}')
                    if 'forward' in position or 'turn' in position or 'backward' in position:
                        # For movement commands, set directly without interpolation
                        self.current_target = position
                        self.interpolating = False
                    else:
                        # For position commands, use smooth interpolation
                        self.smooth_position_transition(position)
                    self.last_demo_action = position
                break

        # Reset demo after completion
        if elapsed > 60:  # Reset after 60 seconds
            self.demo_start_time = current_time
            self.get_logger().info('Demo sequence restarting...')

    def set_target(self, target_name):
        """Public method to set target position"""
        if target_name in self.positions:
            if any(x != 0 for x in self.positions[target_name]['wheels']):
                # Movement command
                self.current_target = target_name
                self.interpolating = False
            else:
                # Position command
                self.smooth_position_transition(target_name)
        else:
            self.get_logger().warn(f'Unknown target: {target_name}')

def main(args=None):
    rclpy.init(args=args)

    try:
        controller = WheelchairController()

        # Add some example commands (you can modify these)
        controller.get_logger().info('Wheelchair controller ready!')
        controller.get_logger().info('Available positions: rest, standing_prep, standing, forward_slow, forward_fast, turn_left, turn_right, backward')

        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
