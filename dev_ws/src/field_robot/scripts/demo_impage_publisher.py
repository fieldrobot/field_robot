#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv_bridge
import tensorflow as tf



class DemoImagePublisher(Node):
    def __init__(self):
        super().__init__('demo_image_publisher')
        self.declare_parameter('image_dst', 'robot_camera_demo')
        
        # ros publisher & subscriber
        self.publisher = self.create_publisher(
            Image,
            self.get_parameter('image_dst').get_parameter_value().string_value,
            10)

        # openCV setup
        self.bridge = cv_bridge.CvBridge()

    def image_callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info("image callback called")

        # convert msg to tensor
        cvImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        tensor = tf.tensor(cvImage.data, [cvImage.rows, cvImage.cols], cvImage)

        # run model
        res = self.model(tensor)
        prediction = self.model.predict(tensor)

        # convert result to msg

        # publish result

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