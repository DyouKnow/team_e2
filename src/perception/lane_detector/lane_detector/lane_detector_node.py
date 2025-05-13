import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None
    
    def compute(self, error):
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.1
        self.last_time = current_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        
        output = (self.Kp * error) + (self.Ki * self.integral) + (self.Kd * derivative)
        return output

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        #PID 설정(감도 조정 가능)
        self.pid = PIDController(Kp=0.004, Ki=0.0, Kd=0.002)

        self.get_logger().info('LaneDetector node started.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = frame.shape
            roi = frame[int(height * 0.6):, :]

            #HSV 변환 및 노란색 마스크
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([35, 255, 255])
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            #컨투어 처리
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            left_xs, right_xs = [], []

            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2

                if cx < width // 2:
                    left_xs.append(cx)
                else:
                    right_xs.append(cx)
                cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,0), 2)

            if left_xs and right_xs:
                left_mean = int(np.mean(left_xs))
                right_mean = int(np.mean(right_xs))
                lane_center = (left_mean + right_mean) // 2
                error = (width // 2) - lane_center

                #PID 제어 계산
                angular_z = self.pid.compute(error)
                linear_x = 0.15 #일정한 속도로 전진

                #속도 명령 퍼블리시
                twist = Twist()
                twist.linear.x = linear_x
                twist.angular.z = angular_z
                self.cmd_pub.publish(twist)

                #시각화(디버깅)
                cv2.line(roi, (left_mean, 0), (left_mean, roi.shape[0]), (255,0,0), 2)
                cv2.line(roi, (right_mean, 0), (right_mean, roi.shape[0]), (0,0,255), 2)
                cv2.line(roi, (lane_center, 0), (lane_center, roi.shape[1]), (255,255,255), 2)
                cv2.line(roi, (width // 2, 0), (width // 2, roi.shape[0]), (0, 255, 255), 1)
                self.get_logger().info(f' Error {error:.2f} | Angular Z: {angular_z:.3f}')

            cv2.imshow("lane Detection", roi)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Image processing Error : {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
