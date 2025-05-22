#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

class DetectYellowLine(Node):
    def __init__(self):
        super().__init__('detect_yellow_line')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('yellow.hue_l', 22),
                ('yellow.hue_h', 32),
                ('yellow.saturation_l', 150),
                ('yellow.saturation_h', 255),
                ('yellow.value_l', 150),
                ('yellow.value_h', 255),
            ]
        )
        self.hue_l = self.get_parameter('yellow.hue_l').get_parameter_value().integer_value
        self.hue_h = self.get_parameter('yellow.hue_h').get_parameter_value().integer_value
        self.sat_l = self.get_parameter('yellow.saturation_l').get_parameter_value().integer_value
        self.sat_h = self.get_parameter('yellow.saturation_h').get_parameter_value().integer_value
        self.val_l = self.get_parameter('yellow.value_l').get_parameter_value().integer_value
        self.val_h = self.get_parameter('yellow.value_h').get_parameter_value().integer_value

        # 구독 & 퍼블리시
        self.sub_image = self.create_subscription(
            Image, '/camera/image_projected', self.image_callback, 1)
        self.pub_mask = self.create_publisher(
            Image, '/detect/yellow_line_mask', 1)
        self.cvBridge = CvBridge()

    def image_callback(self, msg):
        img = self.cvBridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower = np.array([self.hue_l, self.sat_l, self.val_l])
        upper = np.array([self.hue_h, self.sat_h, self.val_h])
        mask = cv2.inRange(hsv, lower, upper)
        mask_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        # rqt_image_view 등에서 확인 가능
        self.pub_mask.publish(self.cvBridge.cv2_to_imgmsg(mask_img, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = DetectYellowLine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
