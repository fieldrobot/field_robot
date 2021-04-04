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
import time
from timeit import default_timer as timer

class EmptySpaceFollowerServer(Node):
    def __init__(self):
        #start node
        super().__init__('headland_turn')
        self.get_logger().info('starting...')

        self.publish = self.create_publisher(Twist, '/cmd_vel', 10)

        '''self.action_server = ActionServer(
            self,
            EmptySpaceFollower,
            'headland_turn',
            self.action_callback
        )'''

        self.get_logger().info('initialized')
        self.action_callback()

    def action_callback(self):#, goal_handle):
        self.get_logger().info('Successfully started headland turn')

        # setting parameters
        self.linear_vel = 0.2 # m/s
        self.row_width = 0.7 # meters
        self.direction = -1 # 1: turn to left row &&& -1: turn to right row

        # calculating additional values
        self.angular_vel = (2 * self.linear_vel) / self.row_width
        self.steer_time = (math.pi * self.row_width)/(2 * self.linear_vel)

        # setting up velocity publishing for movement
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_vel
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.direction * self.angular_vel

        # setting up the timer
        self.end_time = time.perf_counter() + self.steer_time
        # starting movement
        self.publish.publish(twist_msg)

        #actual moving loop
        while time.perf_counter() < self.end_time:
            # if canceled, stop motion here and return canceled
            time.sleep(self.steer_time/20)

        # setting up breaking
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        # breaking after turn
        self.publish.publish(twist_msg)

        '''goal_handle.succeed()
        result = EmptySpaceFollower.Result()
        return result'''

def main():
    rclpy.init()
    follower = EmptySpaceFollowerServer()
    rclpy.spin(follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()