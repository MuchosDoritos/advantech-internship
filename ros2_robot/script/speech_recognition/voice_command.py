"""
Voice Command Handler Node for ROS2 Robot

This node subscribes to voice commands and executes corresponding robot actions.
Currently handles 'forward' and 'stop' commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceCommandHandler(Node):
    """ROS2 Node that processes voice commands and transforms them into robot actions."""
    
    def __init__(self):
        """Initialize the voice command handler node."""
        super().__init__('voice_command_handler')
        
        # Subscribe to voice commands
        self.subscription = self.create_subscription(
            String,
            '/robot/voice_command',
            self.command_callback,
            10
        )
        
        # Publisher for robot control commands
        self.control_publisher = self.create_publisher(
            String,
            '/robot/control',
            10
        )
        
        # Publisher for feedback messages
        self.feedback_publisher = self.create_publisher(
            String,
            '/robot/feedback',
            10
        )
        
        # Command handlers - easily extendable
        self.command_handlers = {
            "forward": self.handle_forward,
            "stop": self.handle_stop
        }
        
        self.get_logger().info('Voice command handler initialized!')

    def command_callback(self, msg):
        """Process incoming voice commands."""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Find and execute the appropriate handler
        handler = self.command_handlers.get(command)
        if handler:
            handler()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def handle_forward(self):
        """Handle the 'forward' command."""
        self.publish_control("forward")
        self.provide_feedback("Moving forward")

    def handle_backward(self):
        """Handle the 'backward' command."""
        self.publish_control("backward")
        self.provide_feedback("Moving backward")

    def handle_left(self):
        """Handle the 'left' command."""
        self.publish_control("left")
        self.provide_feedback("Turning left")

    def handle_right(self):
        """Handle the 'right' command."""
        self.publish_control("right")
        self.provide_feedback("Turning right")
    
    def handle_stop(self):
        """Handle the 'stop' command."""
        self.publish_control("stop")
        self.provide_feedback("Stopping robot")
    
    def publish_control(self, action):
        """Publish a control action to the robot."""
        msg = String()
        msg.data = action
        self.control_publisher.publish(msg)
        self.get_logger().info(f'Published control action: {action}')
    
    def provide_feedback(self, message):
        """Provide feedback about command execution."""
        msg = String()
        msg.data = message
        self.feedback_publisher.publish(msg)
        self.get_logger().debug(f'Feedback sent: {message}')


def main(args=None):
    """Run the voice command handler node."""
    rclpy.init(args=args)
    node = VoiceCommandHandler()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Voice command handler stopped by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()