import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
 
class ImageSubscriber(Node):
  def __init__(self):
    self.counter=[False, False, False, False]
    super().__init__('save_img')
    self.path="/field_robot/src/virtual_maize_field/ai_imgs/"
    self.subscription0 = self.create_subscription(
      Image, 
      '/robot/camera_back/image_raw', 
      self.listener_callback0, 
      10)
    self.subscription1 = self.create_subscription(
      Image, 
      '/robot/camera_front/image_raw', 
      self.listener_callback1, 
      10)
    self.subscription2 = self.create_subscription(
      Image, 
      '/robot/camera_left/image_raw', 
      self.listener_callback2, 
      10)
    self.subscription3 = self.create_subscription(
      Image, 
      '/robot/camera_right/image_raw', 
      self.listener_callback3, 
      10)
    self.subscription0
    self.subscription1
    self.subscription2
    self.subscription3
    self.br = CvBridge()
   
  def listener_callback(self, data, id):
    if self.counter[id]==False:
      self.get_logger().info('Receiving video frame')
      current_frame = self.br.imgmsg_to_cv2(data)
      cv2.imwrite(self.path+time.ctime(time.time())+str(id)+".png", current_frame)
      self.get_logger().info('img saved')
      self.counter[id]=True
    if self.counter[0] and self.counter[1] and self.counter[2] and self.counter[3]:
      self.shutdown()

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
