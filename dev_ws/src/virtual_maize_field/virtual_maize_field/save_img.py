import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import time
 
class ImageSubscriber(Node):
  def __init__(self):
    super().__init__('save_img')
    self.path="/field_robot/src/virtual_maize_field/ai_imgs/"
    self.subscription = self.create_subscription(
      Image, 
      '/robot/camera_back/image_raw', 
      self.listener_callback, 
      10)
    self.subscription = self.create_subscription(
      Image, 
      '/robot/camera_front/image_raw', 
      self.listener_callback, 
      10)
    self.subscription = self.create_subscription(
      Image, 
      '/robot/camera_left/image_raw', 
      self.listener_callback, 
      10)
    self.subscription = self.create_subscription(
      Image, 
      '/robot/camera_right/image_raw', 
      self.listener_callback, 
      10)
    self.subscription
    self.br = CvBridge()
   
  def listener_callback(self, data):
    self.get_logger().info('Receiving video frame')
    current_frame = self.br.imgmsg_to_cv2(data)
    self.get_logger().info('saving to'+self.path+time.ctime(time.time())+str(time.time_ns())+".png")
    cv2.imwrite(self.path+time.ctime(time.time())+str(time.time_ns())+".png", current_frame)
    self.get_logger().info('img saved')
    self.shutdown()

  
def main(args=None):
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()