import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
import sys
 
class ImageSubscriber(Node):
  def __init__(self):
    self.counter=[False, False, False, False]
    self.timestamp = int(time.time())
    self.path="/media/psf/field_robot/imgs"
    super().__init__('save_img')
    self.create_subscription(
      Image, 
      '/robot/camera_back/image_raw', 
      self.listener_callback0, 
      10)
    self.create_subscription(
      Image, 
      '/robot/camera_front/image_raw', 
      self.listener_callback1, 
      10)
    self.create_subscription(
      Image, 
      '/robot/camera_left/image_raw', 
      self.listener_callback2, 
      10)
    self.create_subscription(
      Image, 
      '/robot/camera_right/image_raw', 
      self.listener_callback3, 
      10)
    self.br = CvBridge()
   
  def listener_callback(self, data, id):
    if self.counter[id]==False:
      current_frame = self.br.imgmsg_to_cv2(data)
      current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
      cv2.imwrite(f"{self.path}/{self.timestamp}-{id}.png", current_frame)
      self.get_logger().info(f'Image {id} saved')
      self.counter[id]=True
      if self.counter[0] and self.counter[1] and self.counter[2] and self.counter[3]:
        #self.shutdown()
        rclpy.shutdown()
        #sys.exit()

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
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
