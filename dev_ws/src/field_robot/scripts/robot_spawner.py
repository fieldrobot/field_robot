#!/usr/bin/python3

import os
from tokenize import Pointfloat
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion

def main():
    """ Main for spwaning turtlebot node """

    # Start node
    rclpy.init()
    node = rclpy.create_node("robot_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")

    # Get path to the turtlebot3 burgerbot
    sdf_file_path = os.path.join(
        get_package_share_directory("field_robot"), "models", "robot", "robot.urdf")

    # Set data for request
    request = SpawnEntity.Request()
    request.name = 'robot'
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = 'robot'
    pose = Pose()
    position = Point()
    position.x = -1.14
    position.y = -5.95
    position.z = 0.3
    orientation = Quaternion()
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


if __name__ == "__main__":
    main()