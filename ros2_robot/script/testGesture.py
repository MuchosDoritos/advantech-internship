#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class HandGestureRobotController(Node):
    def __init__(self):
        super().__init__('hand_gesture_robot_controller')
        self.bridge = CvBridge()

        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.get_logger().info('📡 訂閱 /camera/color/image_raw ...')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils

    def count_fingers(self, hand_landmarks):
        """
        根據 MediaPipe 偵測的手部關鍵點來計算豎起的手指數量
        """
        fingers = [0, 0, 0, 0, 0] 

        finger_tips = [4, 8, 12, 16, 20]
        finger_pips = [2, 6, 10, 14, 18]

        for i in range(5):
            if hand_landmarks.landmark[finger_tips[i]].y < hand_landmarks.landmark[finger_pips[i]].y:
                fingers[i] = 1  

        return sum(fingers)  

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            height, width, _ = cv_image.shape

            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            hand_results = self.hands.process(rgb_image)

            finger_count = 0  
            if hand_results.multi_hand_landmarks:
                for hand_landmarks in hand_results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(cv_image, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                    finger_count = self.count_fingers(hand_landmarks)
                    cv2.putText(cv_image, f"Fingers: {finger_count}", (50, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            twist = Twist()
            if finger_count == 0:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("🛑 停止")
            elif finger_count == 1:
                twist.linear.x = 0.2  # 前進
                self.get_logger().info("⬆️ 前進")
            elif finger_count == 2:
                twist.linear.x = -0.2  # 後退
                self.get_logger().info("⬇️ 後退")
            elif finger_count == 3:
                twist.angular.z = 0.5  # 左轉
                self.get_logger().info("🔄 左轉")
            elif finger_count == 4:
                twist.angular.z = -0.5  # 右轉
                self.get_logger().info("🔄 右轉")
            elif finger_count == 5:
                twist.linear.x = 0.5  # 加速前進
                self.get_logger().info("🚀 加速前進")

            self.cmd_vel_publisher.publish(twist)

            if hasattr(cv2, 'imshow'):  
                cv2.imshow('Hand Gesture Control', cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info('🔴 停止手勢控制')
                    rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'影像處理錯誤: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = HandGestureRobotController()
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
