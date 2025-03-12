#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  
            self.scan_callback,
            10)
        self.get_logger().info('📡 LiDAR 訂閱啟動！')

    def scan_callback(self, msg):
        """處理 LiDAR 數據"""
        min_distance = min(msg.ranges)  
        self.get_logger().info(f'🔍 最短距離: {min_distance:.2f} m')

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🔴 停止 LiDAR 訂閱')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


