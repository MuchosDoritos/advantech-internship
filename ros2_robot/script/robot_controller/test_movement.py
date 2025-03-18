#!/usr/bin/env python3
"""
Test Movement Script for Robot

This script moves the robot forward for approximately one meter,
then backward to the starting position.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestMovement(Node):
    """Simple node to test basic robot movement."""
    
    def __init__(self):
        """Initialize the test movement node."""
        super().__init__('test_movement')
        
        # Publisher for robot movement commands
        self.movement_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Standard topic for robot velocity commands
            10
        )
        
        # Set up movement parameters
        self.linear_speed = 0.1  # meters per second (slow for safety)
        self.move_time = 10.0    # seconds (adjust based on robot's speed)
        
        self.get_logger().info('Test movement node initialized!')
        self.get_logger().info('Make sure open_tracer_mini.sh is running in another terminal')
        
        # Create a timer to start the movement sequence
        self.create_timer(1.0, self.start_movement_sequence)
    
    def publish_velocity(self, linear_x, angular_z):
        """Publish velocity command to the robot."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.movement_publisher.publish(twist)
        self.get_logger().info(f'Published velocity: linear_x={linear_x}, angular_z={angular_z}')
    
    def stop_robot(self):
        """Stop the robot."""
        self.publish_velocity(0.0, 0.0)
        self.get_logger().info('Robot stopped')
    
    def start_movement_sequence(self):
        """Start the movement sequence."""
        # This timer callback will only run once
        self.get_logger().info('Starting movement sequence...')
        
        # Forward movement
        self.get_logger().info(f'Moving forward for {self.move_time} seconds...')
        self.publish_velocity(self.linear_speed, 0.0)
        time.sleep(self.move_time)
        
        # Stop
        self.get_logger().info('Stopping...')
        self.stop_robot()
        time.sleep(1.0)  # Pause for 1 second
        
        # Backward movement
        self.get_logger().info(f'Moving backward for {self.move_time} seconds...')
        self.publish_velocity(-self.linear_speed, 0.0)
        time.sleep(self.move_time)
        
        # Final stop
        self.get_logger().info('Stopping...')
        self.stop_robot()
        
        self.get_logger().info('Movement sequence completed!')
        # Shutdown the node after completing the sequence
        self.get_logger().info('Shutting down node...')
        rclpy.shutdown()


def main(args=None):
    """Run the test movement node."""
    rclpy.init(args=args)
    node = TestMovement()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test aborted by user')
        node.stop_robot()  # Ensure robot stops when program is interrupted
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Make sure robot stops before shutting down
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()