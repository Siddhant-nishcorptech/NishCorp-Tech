from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Create launch description for wheelchair control nodes."""

    # Create launch description
    ld = LaunchDescription()

    # Motor controller node
    motor_controller = Node(
        package='wheelchair',
        executable='motor_controller.py',
        name='motor_controller',
        output='screen'
    )

    # Joint controller node
    joint_controller = Node(
        package='wheelchair',
        executable='joint_controller.py',
        name='joint_controller',
        output='screen'
    )

    # Add nodes to launch description
    ld.add_action(motor_controller)
    ld.add_action(joint_controller)

    return ld
