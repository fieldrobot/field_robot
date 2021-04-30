#!/usr/bin/python3

# importing the client library for python: default for every node
import rclpy
# importing the node, necessary to register this script as a node
from rclpy.node import Node
# importing libraries to keep the aciton server running
from rclpy.action import ActionServer, CancelResponse # default action server and making canceling possible
# necessary to execute callbacks at the same time without one blocking the other ones
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# importing message and action files to receive and send data via ROS2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from field_robot.action import BTNode

import math
import time

class HeadlandTurnServer(Node):
    def __init__(self):
        #start node
        super().__init__('headland_turn')
        self.get_logger().info('starting...')

        self.publish = self.create_publisher(Twist, '/cmd_vel', 10)

        self.action_server = ActionServer(
            node=self,
            action_type=BTNode,
            action_name='headland_turn',
            execute_callback=self.action_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('initialized')

    def action_callback(self, goal_handle):
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
        self.end_time = time.perf_counter() + self.steer_time - 0.5
        # starting movement
        self.publish.publish(twist_msg)

        # setting up breaking
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        result = BTNode.Result()

        #actual moving loop
        while time.perf_counter() < self.end_time:
            # if canceled, stop motion here and return canceled
            if goal_handle.is_cancel_requested:
                self.publish.publish(twist_msg)
                goal_handle.canceled()
                return result
            time.sleep(self.steer_time/20)

        # breaking after turn
        self.publish.publish(twist_msg)
        goal_handle.succeed()
        print('returning success')
        return result

    def cancel_callback(self, cancel_requests):
        return CancelResponse.ACCEPT

def main():
    rclpy.init()
    headlandTurn = HeadlandTurnServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(headlandTurn, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()