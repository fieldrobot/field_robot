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

def main():
    """ Main for spwaning turtlebot node """
    rclpy.init()

    node1 = rclpy.create_node('robot_delete')
    node1.declare_parameter('name')

    delete_cli = node1.create_client(DeleteEntity, '/delete_entity')
    node1.get_logger().info("Connecting to `/delete_entity` service...")
    if not delete_cli.service_is_ready():
        delete_cli.wait_for_service()
        node1.get_logger().info("delete-...connected!")

    delete_request = DeleteEntity.Request()
    delete_request.name = node1.get_parameter('name').get_parameter_value().string_value

    future1 = delete_cli.call_async(delete_request)
    rclpy.spin_until_future_complete(node1, future1)
    print("delete completed")

    node1.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()