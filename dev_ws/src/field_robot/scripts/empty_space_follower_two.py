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

        '''sec = msg.header.stamp.secs
        time = msg.header.stamp.nsecs
        time = 1000 * sec + time / 1000000
        dt = time - self.time_old
        # warum sind die ersten beiden Werte falsch? Werkherum:
        if dt > 1000:
            dt = 0
        self.time_old = time'''
        # Frequenz bestimmt durch <update_rate> in  camera.gazebo.xacro
        # wenn die Frequenz zu hoch gewaehlt wird, dann kommt diese Routine nicht hinterher
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, d = image.shape
        # Region of interest, kann durch Beschneidung des Bildes oder andere Kameradefinitionen in der xacro erfolgen
        x1 = int(0)
        y1 = int(0)#int(h/8)
        x2 = int(w)
        y2 = int(h)#int(h - h/8)
        image_cutted = image[y1:y2, x1:x2]
        image_center = w/2
        image_hsv = cv2.cvtColor(image_cutted, cv2.COLOR_BGR2HSV)
        lower_green = numpy.array([30, 100, 0])
        upper_green = numpy.array([80, 255, 255])
        image_filtered = cv2.inRange(image_hsv, lower_green, upper_green)

        image_dilated = cv2.dilate(image_filtered, None, iterations=8)
        
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

        print('streifen', streifen)

        sm = self.streifen_mitte(streifen, image_dilated)

        print('mitte', sm)

        image_with_point = cv2.circle(image_dilated, (int(sm), 50), 20, (255), -20)

        cv2.imshow("kevin", image_with_point)
        
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