#!/usr/bin/env python3

import time
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

 
class ImageSubscriber(Node):
  def __init__(self):
    self.timestamp = int(time.time())
    self.path="/media/psf/field_robot/imgs"
    self.throw_away_amount = 30
    self.throw_away_counter = self.throw_away_amount

    super().__init__('save_img')
    self.create_subscription(Image, '/robot/camera_back/image_raw',     self.listener_callback0, 10)
    self.create_subscription(Image, '/robot/camera_front/image_raw',    self.listener_callback1, 10)
    self.create_subscription(Image, '/robot/camera_left/image_raw',     self.listener_callback2, 10)
    self.create_subscription(Image, '/robot/camera_right/image_raw',    self.listener_callback3, 10)
    self.br = CvBridge()

  def listener_callback(self, data, id):
    if self.throw_away_counter == self.throw_away_amount:
        current_frame = self.br.imgmsg_to_cv2(data)
        current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
        img_path = os.path.join(self.path, f"{int(time.time())}-{id}.png")
        cv2.imwrite(img_path, current_frame)
        self.get_logger().info(f'Image saved at {img_path}')
        self.throw_away_counter = 0
    self.throw_away_counter += 1

  def listener_callback0(self, data):
    self.listener_callback(data, 0)
  def listener_callback1(self, data):
    self.listener_callback(data, 1)
  def listener_callback2(self, data):
    self.listener_callback(data, 2)
  def listener_callback3(self, data):
    self.listener_callback(data, 3)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

if __name__ == '__main__':
    main()