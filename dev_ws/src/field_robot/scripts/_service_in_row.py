#!/usr/bin/python3

# importing libraries used for computation
import cv2
import cv_bridge
import numpy

# necessary to make parallel processes sleeping so that they do not overwork
import time

# importing the client library for python: default for every node
import rclpy
# importing the node, necessary to register this script as a node
from rclpy.node import Node
# importing libraries to keep the aciton server running
from rclpy.action import ActionServer, CancelResponse # default action server and making canceling possible
# necessary to execute callbacks at the same time without one blocking the other ones
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# quality of service profily needs to be created as the image is published not with default configuration
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

# importing message and action files to receive and send data via ROS2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from field_robot.srv import BTNode

# class handling everything concerning empty space following
class ServiceInRow(Node):

    # constructor for the empty space follower
    def __init__(self):
        #starting node and naming it
        super().__init__('service_in_row')

        self.get_logger().info('starting...')
        
        # creating the cv birdge for image processing
        self.bridge = cv_bridge.CvBridge()

        self.in_row = False;

        self.srv = self.create_service(BTNode, 'service_in_row', self.service_callback)

        # declaring the qos_profile for image subscription
        self._qos_profile = QoSProfile(
            history=QoSHistoryPolicy.SYSTEM_DEFAULT
        )
        self._qos_profile.history = QoSHistoryPolicy.SYSTEM_DEFAULT
        self._qos_profile.depth = 10
        self._qos_profile.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
        self._qos_profile.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
        # subscribing the image and declaring callbacks and the qos profile
        self.subscription = self.create_subscription(
            msg_type=Image, # the message file the received data has
            topic='field_robot/gazebo_cam/image_raw', # the topic to be subscribed
            callback=self.image_callback, # declaring the callback for image processing 
            callback_group=ReentrantCallbackGroup(), # making parallel execution possible my multithreading
            qos_profile=self._qos_profile # the qos profile
        )

        self.get_logger().info('initialized')

    def service_callback(self, request, response):
        response.confirm = self.in_row
        return response

    # the image processing/actual navigation
    def image_callback(self, msg):
        print('image callback')
        #### image preparation

        # convertin image to opencv2
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, d = image.shape

        # convertin cv2 image to hsv
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # filtering image for green corn
        lower_green = numpy.array([30, 100, 0])
        upper_green = numpy.array([80, 255, 255])
        image_filtered = cv2.inRange(image_hsv, lower_green, upper_green)

        # dilating image: making everything more bold -> flatten
        image_dilated = cv2.dilate(image_filtered, None, iterations=8)

        if not numpy.any(image_dilated):
            self.in_row = False
        else:
            self.in_row = True

def main():
    rclpy.init()
    service = ServiceInRow()
    executor = MultiThreadedExecutor()
    rclpy.spin(service, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()