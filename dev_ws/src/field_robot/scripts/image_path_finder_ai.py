#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
import numpy
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
            (self.get_parameter('image_src').get_parameter_value().string_value),
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            (self.get_parameter('image_dst').get_parameter_value().string_value),
            10)
        self.subscription

        # openCV setup
        self.bridge = cv_bridge.CvBridge()

        # tensorflow setup
        string = "TensorFlow version: " + tf.__version__
        self.get_logger().info(string)
        print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))
        models_dir = os.path.join(get_package_share_directory('field_robot'), 'config', 'ai_models', 'image_path_finder_ai')
        self.model = tf.keras.models.load_model(models_dir)

        # timer setup
        self.time = self.get_clock().now()

    def image_callback(self, msg):
        # prints
        self.get_logger().info("image callback called")
        self.get_logger().info(str(((self.get_clock().now() - self.time).nanoseconds) / 1000000000))
        self.time = self.get_clock().now()

        # convert msg to tensor
        cvImage = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_array = numpy.asarray(cvImage)
        #self.get_logger().info(str(image_array.shape))
        image_array = numpy.expand_dims(image_array, 0)
        image_array = image_array/255.
        #self.get_logger().info(str(numpy.amax(image_array)))
        tensor = tf.convert_to_tensor(image_array)


        # run model
        prediction = self.model.predict(tensor)
        predictionO = (prediction[0]*255.).astype(numpy.uint8)
        #self.get_logger().info("prediction0 shape" + str(predictionO.shape))
        #self.get_logger().info("prediction0" + str(predictionO))

        # convert result to msg
        after = self.bridge.cv2_to_imgmsg(predictionO, encoding='mono8')
        after.header = msg.header

        # publish result
        self.publisher.publish(after)

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