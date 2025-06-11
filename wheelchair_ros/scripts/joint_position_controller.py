#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointPositionController(Node):
    def __init__(self):
        super().__init__('joint_position_controller')

        # Preset positions
        self.positions = {
            'closed': {
                'j1-j5': [0.0, 0.0, 0.0, 0.0, 0.0],
                'wheels': [0.0, 0.0, 0.0, 0.0]
            },
            'standing': {
                'j1-j5': [-1.50, 0.50, 0.50, -1.00, 1.00],
                'wheels': [0.0, 0.0, 0.0, 0.0]
            },
            'moving_forward': {
                'j1-j5': [0.0, 0.0, 0.0, 0.0, 0.0],
                'wheels': [2.0, 2.0, 2.0, 2.0]  # Velocity for all wheels (rad/s)
            }
        }

        # Publisher for position commands
        self.position_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10
        )

        # Publisher for wheel commands
        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/wheel_controller/commands',
            10
        )

        # Timer to publish commands
        self.timer = self.create_timer(0.1, self.publish_commands)  # Increased frequency to 10 Hz
        self.target_position = 'closed'  # Initial position

    def publish_commands(self):
        """Publish joint commands for current target position"""
        pos = self.positions[self.target_position]

        # Publish position commands
        position_msg = Float64MultiArray()
        position_msg.data = pos['j1-j5']
        self.position_pub.publish(position_msg)

        # Publish wheel commands
        wheel_msg = Float64MultiArray()
        wheel_msg.data = pos['wheels']
        self.wheel_pub.publish(wheel_msg)

        self.get_logger().info(f'Published {self.target_position} position')

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionController()

    # Switch to standing position after 5 seconds
    node.create_timer(5.0, lambda: setattr(node, 'target_position', 'standing'))
    # Switch to moving_forward after 10 seconds
    node.create_timer(10.0, lambda: setattr(node, 'target_position', 'moving_forward'))

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
