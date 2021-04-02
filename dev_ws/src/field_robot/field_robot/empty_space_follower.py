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

import cv2
import cv_bridge
import numpy
import math
from time import sleep
from timeit import default_timer as timer

class Follower(Node):
    def __init__(self):
        #start node
        super().__init__('empty_space_follower')
        #node = rclpy.create_node("empty_space_follower")
        self.bridge = cv_bridge.CvBridge()

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
        self.publish = self.create_publisher(Twist, '/cmd_vel', 10)
        print('bu')

    def image_callback(self, msg):
        #start
        # self.start = timer()
        # end_time = timer()
        # time = 1000 * (end_time - self.start)
        # print(time)
        print('hi')
        '''sec = msg.header.stamp.secs
        time = msg.header.stamp.nsecs
        time = 1000 * sec + time / 1000000
        dt = time - self.time_old
        # warum sind die ersten beiden Werte falsch? Werkherum:
        if dt > 1000:
            dt = 0
        self.time_old = time
        # Frequenz bestimmt durch <update_rate> in  camera.gazebo.xacro
        # wenn die Frequenz zu hoch gewaehlt wird, dann kommt diese Routine nicht hinterher
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # cv2.imshow("original image", image)
        # Bildgroesse in camera.gazebo.xacro definiert mit w = 320, h = 240 Pixel
        # <width>320</width> <height>240</height>
        h, w, d = image.shape
        # Region of interest, kann durch Beschneidung des Bildes oder andere Kameradefinitionen in der xacro erfolgen
        x1 = 0
        y1 = h/4
        x2 = w
        y2 = h - h/5
        image_cut = image[y1:y2, x1:x2]
        image_center = w/2
        hsv = cv2.cvtColor(image_cut, cv2.COLOR_BGR2HSV)
        # cv2.imshow("hsv corn", hsv)
        lower_green = numpy.array([30, 100, 0])
        upper_green = numpy.array([80, 255, 255])
        thresh = cv2.inRange(hsv, lower_green, upper_green)
        # weisse Pixel als Begrenzung, ist fuer die for-Schleife unte noetig
        # kann aber auch modifiziert werden, um z.B. as Bild schraeg zuzuschneiden
        for row in range(len(thresh)):
            thresh[row][1] = 255
            thresh[row][len(thresh[0]) - 2] = 255
        # cv2.imshow("green corn", thresh)
        self.start = timer()
        # Bildzeilen nach den jeweils laengsten schwarzen Pixelreihen durchsuchen
        av_path = 0
        cnt = 0
        white_pix_cnt = 0
        search_white_pix = False
        for row in range(len(thresh)):
            hi_flag = False
            first_lo = 0
            path_len = 0
            path = [0, 0]
            for i, y in enumerate(thresh[row]):
                if y > 0:  # wenn Mais
                    white_pix_cnt += 1
                    if not hi_flag:
                        hi_flag = True
                        first_hi = i
                        if first_hi - first_lo > path_len:
                            path_len = first_hi - first_lo
                            path = [i, path_len]
                else:
                    if hi_flag:
                        hi_flag = False
                        first_lo = i

            path_x = path[0] - path[1]/2
            if path_x > 0:
                av_path = av_path + path_x
                cnt += 1

        end_time = timer()
        time = 1000 * (end_time - self.start)
        print('loop time',time)

        # Mittelwert, kann durch zusaetzlichen Code auch je nach Linie gewichtet werden
        # z.B. in der Ferne und/oder ganz vorn weniger
        offset = 1
        if cnt > 0:
            av_path = av_path/cnt - offset
        # print('av_path', av_path)

        # wenn praktisch nichts ausser den selbst geweissten Pixeln (z.B. 2 * h) zu sehen ist,
        # entweder halten oder drehen
        border_pix_num = 2 * h
        min_pix_num = h/2
        min_pix_num = border_pix_num + min_pix_num
        if white_pix_cnt < min_pix_num:
            # print(white_pix_cnt)
            search_white_pix = True

        # PI-Regler
        kp = 0.005
        ki = 0.0001
        turn_speed = 0.2
        linear_speed = 0.05
        error = image_center - av_path
        iterm_max = 100
        self.i_error_sum += error * dt/1000  # dt in von ms in s
        self.i_error_sum = constrain(self.i_error_sum, -iterm_max, iterm_max);
        # print(self.i_error_sum)
        control = kp * error + ki * self.i_error_sum
        print(av_path, error, control, kp * error, ki * self.i_error_sum)
        # wenn nichts zu sehen ist, erst einmal sondieren turn_speed
        # search_white_pix = False
        if search_white_pix:
            print('search')
            self.twist.linear.x = 0
            self.twist.angular.z = turn_speed
        else:
            # wenn twist.linear.x auf Null gesetzt wird, dann dreht der Roboter auf der Stelle,
            # bis im Bild das Ziel in der Bildmitte liegt
            # linear_speed = 0
            self.twist.linear.x = linear_speed
            self.twist.angular.z = control
        self.cmd_vel_pub.publish(self.twist)

        # Mitte des Bildes unten markieren
        cv2.line(image, (image_center, h - 10), (image_center, h), (255, 255, 255), 2)
        # Ziel markieren
        cv2.circle(image, (av_path, h - 20), 5, (255, 255, 255), 2)
        # originales Bild zeigen, andere sind oben auskommentiert
        cv2.imshow("corn rows", image)
        cv2.waitKey(1)'''

def main():
    rclpy.init()
    follower = Follower()
    rclpy.spin(follower)
    rclpy.shutdown()

if __name__ == '__main__':
    main()