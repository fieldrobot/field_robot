#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
import cv_bridge
import tensorflow as tf



class DemoImagePublisher(Node):
    def __init__(self):
        super().__init__('demo_image_publisher')
        self.declare_parameter('image_src')
        self.declare_parameter('sub_topic')
        self.declare_parameter('image_dst')
        
        # ros publisher & subscriber
        self.publisher = self.create_publisher(
            Image,
            (self.get_parameter('image_dst').get_parameter_value().string_value),
            10)

        # openCV setup
        self.bridge = cv_bridge.CvBridge()

        # timer
        #timer_period = 1./30.
        #self.get_logger().info(str(timer_period))
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscriber = self.create_subscription(
            Image,
            (self.get_parameter('sub_topic').get_parameter_value().string_value),
            self.image_callback,
            10)

    def image_callback(self, msg):
        cv_image = cv2.imread(self.get_parameter('image_src').get_parameter_value().string_value)
        ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        ros_image.header = msg.header
        self.publisher.publish(ros_image)

def main(args=None):
    # Start node
    rclpy.init()

    node = DemoImagePublisher()
    rclpy.spin(node)

    # shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()