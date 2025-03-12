#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('📡 Starting movement node...')

        # Define motion parameters
        self.forward_duration = 2.0  # seconds needed to cover ~1 meter at given speed
        self.backward_duration = 2.0
        self.speed = 0.5             # linear speed (m/s) - adjust as needed
        
        # Start with forward movement
        self.phase = 'forward'
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Timer to control the movement loop
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time
        twist = Twist()

        if self.phase == 'forward':
            if elapsed < self.forward_duration:
                twist.linear.x = self.speed
                self.get_logger().info('Moving forward...')
            else:
                self.phase = 'backward'
                self.start_time = current_time  # reset timer for backward phase
                self.get_logger().info('Switching to backward movement')
        elif self.phase == 'backward':
            if elapsed < self.backward_duration:
                twist.linear.x = -self.speed
                self.get_logger().info('Moving backward...')
            else:
                twist.linear.x = 0.0
                self.publisher_.publish(twist)
                self.get_logger().info('Movement complete. Shutting down node.')
                rclpy.shutdown()
                return

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MoveRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
