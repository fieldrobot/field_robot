import os
import cv2
import cv_bridge
import numpy
import sys
from sys import exit
import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from sensor_msgs.msg import Image
from skimage import measure
from timeit import default_timer as timer
from nav_msgs.msg import Odometry
import std_msgs.msg

class PointCloud2FromCamera(Node):
    def __init__(self):
        print('init')
        super().__init__('point_cloud2_from_camera')
        self.subscription = self.create_subscription(
            Image,
            'image',
            self.image_callback,
            10
        )
        self.subscription
    
    def image_callback(self, msg):
        print('image callback')
        #Start der Zeitmessung zur Bestimmung der Rechenzeit
        start_ = timer()

        #Bild von ROS zu OpenCV umwandeln
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #Farmraum von RGB zur HSV umwandeln
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #Bild nach den organgen Blobs filtern: die Filtergrenzen festlegen
        lower_orange = numpy.array([0, 90, 81])
        upper_orange = numpy.array([45, 255, 255])
        #Bild nach den organgen Blobs filtern: das eigentliche Filtern
        thresh = cv2.inRange(hsv, lower_orange, upper_orange)
        #Erode und Dilate zur Glaettung der Kanten und zum Entfernen von stoerendem Rauschen
        thresh = cv2.erode(thresh, None, iterations=1)
        thresh = cv2.dilate(thresh, None, iterations=1)
        #Alle getrennten Bereiche des Bildes (der Hintergrund und die Blobs) werden erkannt und all zu einem Bereich gehoerenden Pixel werden auf jeweils einen spezifischen Wert gesetzt.
        labels = measure.label(thresh)
        #mask = numpy.zeros(thresh.shape, dtype="uint8") UNNOETIG: zum Testen und Ueberrest von frueheren Versionen
        #Alle spezifische Werte werden ermittelt. Jeder taucht in diesem Array nur noch EINMAL auf.
        blobs = numpy.unique(labels)

def main(args=None):
    print('main')
    rclpy.init(args=args)

    pointCloud2FromCamera = PointCloud2FromCamera()

    rclpy.spin(pointCloud2FromCamera)

    pointCloud2FromCamera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()