import os
import sys

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
import rclpy.time
from rclpy.duration import Duration

from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import std_msgs.msg
from field_robot.action import EmptySpaceFollower

import cv2
import cv_bridge
import numpy
import math
from time import sleep
from timeit import default_timer as timer

class EmptySpaceFollowerServer(Node):
    def __init__(self):
        #start node
        super().__init__('empty_space_follower')
        self.get_logger().info('starting...')
        
        self.bridge = cv_bridge.CvBridge()

        self.working = True

        self.publish = self.create_publisher(Twist, '/cmd_vel', 10)

        self.action_server = ActionServer(
            self,
            EmptySpaceFollower,
            'empty_space_follower',
            self.action_callback
        )

        self._qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_ALL
        )
        self._qos_profile.history = QoSHistoryPolicy.SYSTEM_DEFAULT
        self._qos_profile.depth = 10
        self._qos_profile.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
        self._qos_profile.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
        self.subscription = self.create_subscription(
            msg_type=Image,
            topic='field_robot/gazebo_cam/image_raw',
            callback=self.image_callback,
            qos_profile=self._qos_profile
        )

        self.get_logger().info('initialized')

    def action_callback(self, goal_handle):
        self.get_logger().info('Setting goal...')
        self.working = True
        self.get_logger().info('Successfully initiated navigation')
        goal_handle.succeed()
        result = EmptySpaceFollower.Result()
        return result

    def image_callback(self, msg):
        self.get_logger().info('Calling image callback...')
        print(msg.header.stamp)
        if self.working == False:
            self.get_logger().info('Inactive')
            return
        self.get_logger().info('Starting image processing...')

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
        
        #### searching the biggenst stripe without any plante
        streifen = 50
        max = w
        min = 0

        while (max-min) > 2:
            if self.mindestens_einmal(streifen, image_dilated):
                min = streifen
                streifen = streifen + ((max - min)/2)
            else:
                max = streifen
                streifen = streifen - ((max - min)/2)

        #### calculating the x axis middle of the biggest stripe
        sm = self.streifen_mitte(streifen, image_dilated)

        #### visualizing that middle
        image_with_point = cv2.circle(image_dilated, (int(sm), 50), 20, (255), -20)
        cv2.imshow("kevin", image_with_point)
        
        #### regulating the velocity command
        #calculation the error
        error = (w/2)-sm #negative: to far to the left -> right &&&&&&& positive: to far to the right -> left

        #preparing the message to be send
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        #targets/configuaration
        linear_target = 0.2
        angular_d = 0.002
        linear_d = 1

        #p-control
        twist_msg.angular.z = error * angular_d
        #twist_msg.linear.x = linear_target + linear_d / error

        #publishing the velocity commands
        self.publish.publish(twist_msg)

        cv2.waitKey(1)

    def mindestens_einmal(self, streifen, image):
        count = 0
        array = []
        image_t = image.T
        #cv2.imshow("oho",image_t)
        #cv2.imshow("oha",image)
        for column in image_t:
            if not numpy.any(column):
                count = count+1
                #print('0')
            elif count != 0:
                if count >= streifen:
                    array.append(count)
                count = 0
                #print('1')
            #else:
                #print('1')
        if count != 0 and count >= streifen: array.append(count)
        if len(array) >= 1: return True
        return False

    def streifen_mitte(self, streifen, image):
        streifen = streifen - 10
        iterator = 0
        start = 0
        image_t = image.T
        for column in image_t:
            if numpy.any(column) and (iterator-start) > streifen:
                return (iterator + start)/2
            elif numpy.any(column):
                start = iterator
            iterator = iterator+1 
        if (iterator-start) > streifen: return (iterator + start)/2
        print('fuck')
        return iterator/2

def main():
    rclpy.init()
    follower = EmptySpaceFollowerServer()
    rclpy.spin(follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()