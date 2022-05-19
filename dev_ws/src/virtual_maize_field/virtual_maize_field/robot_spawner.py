#!/usr/bin/python3

import time
import random
import os
from tokenize import Pointfloat
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import DeleteEntity
from geometry_msgs.msg import Pose, Point, Quaternion

import numpy as np
import random

from rclpy.node import Node

def main():
    """ Main for spwaning turtlebot node """
    rclpy.init()

    # Start node
    node = rclpy.create_node("robot_spawner")

    node.declare_parameter('x')
    node.declare_parameter('y')
    node.declare_parameter('z')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the turtlebot3 burgerbot
    sdf_file_path = os.path.join(
        get_package_share_directory("virtual_maize_field"), "models", "robot", "robot.urdf")

    # Set data for request
    request = SpawnEntity.Request()
    request.name = 'robot'
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = 'robot'
    pose = Pose()
    position = Point()
    position.x = node.get_parameter('x').get_parameter_value().string_value
    position.y = node.get_parameter('y').get_parameter_value().string_value
    position.z = node.get_parameter('z').get_parameter_value().string_value
    #position.x = -1.14
    #position.y = -5.95
    #position.z = 0.3
    orientation = Quaternion()
    #

    arr = get_quaternion_from_euler(0, 0, (random.random()*2*np.pi))

    #
    orientation.x = 0.0
    orientation.y = 0.0
    orientation.z = 0.682
    orientation.w = 0.732
    pose.position = position
    pose.orientation = orientation
    request.initial_pose = pose

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

if __name__ == "__main__":
    main()