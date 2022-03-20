#!/usr/bin/python3

import os
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

class ImageAIPathFinder(Node):
    def __init__(self):
        super().__init__("image_path_finder_ai")
        self.subscription = self.create_subscription(
            Image,
            'topic',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            'topic',
            10)
        self.subscription

    def image_callback(self, msg):
        self.publisher.publish(msg)
        self.get_logger().info("image callback called")

def main(args=None):
    # Start node
    rclpy.init(args)

    node = ImageAIPathFinder()
    rclpy.spin(node)

    # shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()