#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class YellowLineCenterPub(Node):
    def __init__(self):
        super().__init__('yellow_line_center_pub')
        self.sub_mask = self.create_subscription(
            Image, '/detect/yellow_line_mask', self.mask_callback, 1)
        self.pub_center = self.create_publisher(
            Float64, '/detect/lane', 1)
        self.cvBridge = CvBridge()

    def mask_callback(self, msg):
        mask = self.cvBridge.imgmsg_to_cv2(msg, 'bgr8')
        mask_gray = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        indices = np.where(mask_gray > 127)
        if len(indices[0]) == 0:
            return  # 선을 못찾음
        mean_x = np.mean(indices[1])
        # 원하는 경우, y(세로) 위치를 제한하여 바닥에 가까운 부분만 중심 계산 가능
        center_msg = Float64()
        center_msg.data = mean_x
        self.pub_center.publish(center_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YellowLineCenterPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
