"""
Robot Controller Node for ROS2

This node subscribes to voice commands and translates them
into movement commands for the robot via cmd_vel topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class RobotController(Node):
    """ROS2 Node that controls robot movement based on voice commands."""
    
    def __init__(self):
        """Initialize the robot controller node."""
        super().__init__('robot_controller')
        
        # Subscribe to voice commands
        self.subscription = self.create_subscription(
            String,
            '/robot/voice_command',
            self.command_callback,
            10
        )
        
        # Publisher for robot movement commands
        self.movement_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # This matches your robot's expected topic
            10
        )
        
        # Create a timer for stopping the robot after timeout
        self.stop_timer = None
        self.movement_timeout = 7.0  # Robot stops after 3 seconds if no new command
        
        # Define movement speeds - adjust these based on your robot's capabilities
        self.linear_speed = 0.15  # m/s - reduced for safety during testing
        self.angular_speed = 0.3  # rad/s - reduced for safety during testing
        
        # Movement state tracking
        self.current_movement = "stopped"
        
        self.get_logger().info('Robot controller initialized and publishing to /cmd_vel!')
        self.get_logger().info('Make sure open_tracer_mini.sh is running in another terminal')
    
    def command_callback(self, msg):
        """Process incoming voice commands and control the robot."""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Handle different movement commands
        if command == "forward":
            self.move_forward()
        elif command == "backward":
            self.move_backward()
        elif command == "left":
            self.turn_left()
        elif command == "right":
            self.turn_right()
        elif command == "stop":
            self.stop_robot()
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def move_forward(self):
        """Command the robot to move forward."""
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
        self.movement_publisher.publish(twist)
        self.current_movement = "forward"
        self.get_logger().info('Moving forward')
        self.set_stop_timer()
    
    def move_backward(self):
        """Command the robot to move backward."""
        twist = Twist()
        twist.linear.x = -self.linear_speed
        twist.angular.z = 0.0
        self.movement_publisher.publish(twist)
        self.current_movement = "backward"
        self.get_logger().info('Moving backward')
        self.set_stop_timer()
    
    def turn_left(self):
        """Command the robot to turn left."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        self.movement_publisher.publish(twist)
        self.current_movement = "left"
        self.get_logger().info('Turning left')
        self.set_stop_timer()
    
    def turn_right(self):
        """Command the robot to turn right."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = -self.angular_speed
        self.movement_publisher.publish(twist)
        self.current_movement = "right"
        self.get_logger().info('Turning right')
        self.set_stop_timer()
    
    def stop_robot(self):
        """Command the robot to stop."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.movement_publisher.publish(twist)
        self.current_movement = "stopped"
        self.get_logger().info('Stopping robot')
        self.cancel_stop_timer()
        
        # Send multiple stop commands to ensure robot stops
        # (sometimes a single stop command might be missed)
        self.create_timer(0.1, self.repeat_stop, callback_group=None)
    
    def repeat_stop(self):
        """Send a second stop command to ensure robot stops."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.movement_publisher.publish(twist)
    
    def set_stop_timer(self):
        """Set a timer to stop the robot after a timeout."""
        self.cancel_stop_timer()
        self.stop_timer = self.create_timer(self.movement_timeout, self.stop_timeout)
    
    def cancel_stop_timer(self):
        """Cancel the stop timer if it exists."""
        if self.stop_timer:
            self.stop_timer.cancel()
            self.stop_timer = None
    
    def stop_timeout(self):
        """Stop the robot after timeout expires."""
        self.get_logger().info('Movement timeout reached, stopping robot')
        self.stop_robot()
        self.stop_timer = None


def main(args=None):
    """Run the robot controller node."""
    rclpy.init(args=args)
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Robot controller stopped by user')
        # Send a final stop command before shutting down
        node.stop_robot()
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()