#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class CameraYoloDepthSubscriber(Node):
    def __init__(self):
        super().__init__('camera_yolo_depth_subscriber')
        self.bridge = CvBridge()

        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.get_logger().info('📡 開始訂閱 /camera/color/image_raw ...')

        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        self.get_logger().info('📡 開始訂閱 /camera/depth/image_rect_raw ...')

        self.latest_depth_image = None

        self.distance_threshold = 1.0 

        self.model = YOLO('yolov8l.pt')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            results = self.model(cv_image, conf=0.5)[0]

            height, width, _ = cv_image.shape

            for result in results.boxes.data:
                x1, y1, x2, y2, conf, cls = result.cpu().numpy()
                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])

                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2

                depth = self.get_depth_at_point(center_x, center_y)

                color = (0, 255, 0)  
                if depth > 0 and depth < self.distance_threshold:
                    color = (0, 0, 255)  


                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 2)
                cv2.putText(cv_image, f"{self.model.names[int(cls)]} {depth:.2f}m",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                self.get_logger().info(f"🆔 {self.model.names[int(cls)]} | 位置: ({center_x}, {center_y}) | 距離: {depth:.2f}m")

            cv2.imshow('YOLO Depth Estimation', cv_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('🔴 停止辨識節點')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'影像處理錯誤: {str(e)}')

    def depth_callback(self, msg):
        """接收深度影像，轉換成 NumPy 陣列"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度影像處理錯誤: {str(e)}')

    def get_depth_at_point(self, x, y):
        """從最新的深度影像中獲取某點的深度值"""
        if self.latest_depth_image is None:
            return -1  

        depth = self.latest_depth_image[y, x] / 1000.0 
        return depth

def main(args=None):
    rclpy.init(args=args)
    node = CameraYoloDepthSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🔴 停止訂閱節點')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()