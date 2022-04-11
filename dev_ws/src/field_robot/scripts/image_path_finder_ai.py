#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
import cv_bridge
import tensorflow as tf



class ImageAIPathFinder(Node):
    def __init__(self):
        super().__init__('image_path_finder_ai')
        self.declare_parameter('image_src')
        self.declare_parameter('image_dst')
        
        # ros publisher & subscriber
        self.subscription = self.create_subscription(
            Image,
            (self.get_namespace() + self.get_parameter('image_src').get_parameter_value().string_value),
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            (self.get_namespace() + self.get_parameter('image_dst').get_parameter_value().string_value),
            10)
        self.subscription

        # openCV setup
        self.bridge = cv_bridge.CvBridge()

        # tensorflow setup
        string = "TensorFlow version: " + tf.__version__
        self.get_logger().info(string)
        models_dir = os.path.join(get_package_share_directory('field_robot'), 'config', 'ai_models', 'image_path_finder_ai')
        self.model = tf.keras.models.load_model(models_dir)

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

    node = ImageAIPathFinder()
    rclpy.spin(node)

    # shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()