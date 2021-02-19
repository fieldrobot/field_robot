import os
import cv2
import cv_bridge
import numpy
import sys
from sys import exit
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

import tf2_ros

from ament_index_python.packages import get_package_share_directory
from timeit import default_timer as timer
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import std_msgs.msg

class PointCloud2FromCamera(Node):
    def __init__(self):
        print('init')
        super().__init__('point_cloud2_from_camera')

        # general parameter definition
        self.goalFrame = 'base_footprint' #hier base_footprint und nicht base_link, das sich base_link bei unserem Roboter spaeter neigt.
        self.cameraFrame = 'camera' #Frame des Kamerabildes
        self.pointCloudTopic = '/camera/blob_cloud' #topic, auf welchem die PointCloud2 publiziert werden soll

        # camera parameter definition: das habe ich berechnet. Wie genau fuehre ich hier nicht aus, da dann der Tag vorbei waere ;)
        self.pixelSize = 0.003257794325
        self.focalLength = 1
        self.height = 240
        self.width = 420

        # publisher and subscriber definition
        self.tf2Buffer = tf2_ros.Buffer()
        self.tf2Listener = tf2_ros.TransformListener(self.tf2Buffer)
        #self.tfListener = tf2_ros.TransformListener(self.tfBuffer) #Hiermit kann ich die Transformationen zwischen den einzelnene Frames abfragen.
        #self.tfListener.waitForTransform(self.goalFrame, self.cameraFrame, rospy.Time(0), rospy.Duration(3)) #Hier wird gewartet, bis ich Transformationen empfangen kann. Ansonsten kommt eine Fehlermeldung in der Art von: noch nicht
        #self.pub = rospy.Publisher(self.pointCloudTopic, PointCloud2, queue_size=10) #Hiermit wird nachher die PointCloud2 publiziert
        time.sleep(2)
        ##TEST##
        transform_result = self.tf2Buffer.lookup_transform(self.goalFrame, self.cameraFrame, self.get_clock().now())
        print(transform_result)
        ##TEST END##

        #qos_profile = QoSProfile()
        #qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT
        self.subscription = self.create_subscription(
            Image,
            'field_robot/gazebo_cam/Image_raw',
            self.image_callback,
            10
        )
        self.bridge = cv_bridge.CvBridge()
        self.subscription
    
    def image_callback(self, msg):
        print('image callback')
        #Start der Zeitmessung zur Bestimmung der Rechenzeit
        start_ = timer()
        #Erstelung eines Objektes des selbst definierten Dateiformates zur Blob-Publizierung. Hier werden die Daten reingeschrieben. Das Objekt wird nachher publiziert.
        blobsMes = []


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
        #Jede erkannte Region kann nun ein Blob sein und wird deswegen weiter getrennt betrachtet.
        for label in blobs:
            #Hiermit soll der Hintergund aussortiert werden. Optimierungsbedarf: ist ein Blob links oben im Bild, wird dieser aussortiert.
            if label == 0:
                continue
            #Hier wird ein leeres "Bild" erzeugt, mit Hilfe welchem, die eine Region (der eine Blob) betrachtet werden soll.
            labelMask = numpy.zeros(thresh.shape, dtype="uint8")
            #Alle zum Bereich gehoerenden Pixel werden weiss gefaerbt. Denn label gibt die aktuelle Bereichnummer an und labels ist die Maske, in welcher alle Bereiche markiert sind.
            labelMask[labels == label] = 255
            # numPixels = cv2.countNonZero(labelMask) UNNOETIG: zum Testen und Ueberrest von frueheren Versionen
            # if numPixels > 30: UNNOETIG: zum Testen und Ueberrest von frueheren Versionen
            #mask = cv2.add(mask, labelMask) UNNOETIG: zum Test und Ueberrest von frueheren Versionen
            #Berechnung des Mittelpunktes von EINEM Blob
            uniqueM = cv2.moments(labelMask)
            uniqueX = int(uniqueM["m10"] / uniqueM["m00"])
            uniqueY = int(uniqueM["m01"] / uniqueM["m00"])

            blobsMes.append([uniqueX, uniqueY])

        ##### NOW THE POINT DETECTION HAS FINISEHD AND THE DATA IS PUBLISHED #####
        points = [] #in diesem Array werden die einzelnen Punkt in 3D-gespeichert

        for singleBlob in blobsMes:
            #Hier werden die Transformationsaenerung und Rotationsaenderungen von dem Kamera-Frame zum Roboter-Frame berechnet. Die Funktion ist von ROS gegeben.
            transform_result = self.tf2Buffer.lookupTransform()
            #(transform_from_cam, rotation_from_cam) = self.tfListener.lookupTransform(self.goalFrame, self.cameraFrame, rospy.Time(0))
            #Ergebnis: Pixel-Bild-Koordinate in ROS-Koordinate (in m) ungewandelt (Beschreibung siehe bei der Funktion9
            cam_coordinate_unrotated = self.pixel_to_point(singleBlob[0], singleBlob[1])
            #aus der einfachen 3D-Koordinate, die bisher keinerlei Drehungen der Kamera beruecksichtigt, wird hier nun eine neue Koordinate generiert, die die Drehung beruecksichtigt.
            #Somit zeigen die Koordinaten von dem Frame der Kamera aus nun in die richtige Richtung.
            cam_coordinate = self.qv_mult(rotation_from_cam, cam_coordinate_unrotated)

            #es werden ausschliesslich Koordinaten behandelt, welche aus Sicht der Kamera nach unten zeigen. Deren z-Kooridnate also <0 ist.
            if cam_coordinate[2] < 0:
                #Jede ermittelte Koordinate ist ja nur eine von unendlich vielen moeglichen Punkten eines Blobs. Dabei liegen alle Punkte auf einer Gerade.
                #Hier wird jetzt die Koordinate dieser Gerade germittelt, die auf dem Boden liegt und somit den echten Blob darstellt.
                cam_coordinate_ground = self.ground_coordinate(cam_coordinate, transform_from_cam)
                #Hier wird die Kooridnate zu angepasst, dass sie sich nun auf einen anderen Bezugsframe(base_footprint, anstelle von camera) bezieht.
                link_coordinate = self.transform_coordinate(cam_coordinate_ground, transform_from_cam)

                #points.append([0.5, 0.5, 0])
                #die ermittelte Koordiante wird nun dem Array aller Punkte hinzugefuegt.
                points.append([link_coordinate[0], link_coordinate[1], link_coordinate[2]])


    #ein gegebenes Pixelpaar wird in eine ROS 3D-Koordinate umgewandelnt
    def pixel_to_point(self, x_im, y_im):
        #Verschiebung von (0,0) von der linken oberen Ecke des Bildes in die Mitte des Bildes. Dies entspricht der Koordinatendarstellung.
        x_im = -x_im + (self.width / 2)
        y_im = -y_im + (self.height / 2)
        #Die Pixel werden nun mit der Pixelgroesse von Pixel in m umgerechnet, um als Koordinate angegeben werden zu koennen.
        y_co = x_im * self.pixelSize
        z_co = y_im * self.pixelSize
        #Abhaengig von der verwendeten Brennweite, wird hier die x-Koordinate fest gesetzt.
        x_co = 1
        #Der ermittelte 3D-Punkt fuer den uebergebenen Pixel wird zurueckgegeben.
        return numpy.array([x_co, y_co, z_co])
    
    #Rotationen werden in ROS in Quaternions statt in xyz-Rotationen angegebne. Das hat verschiedenste Vorteile, u. A. sind die Bewegungen von Robotern eleganter.
    #Siehe z. B. Gimbal Lock auf Wikipedia. Der einzige Nachteil von Quaternions: Sie sind ein 4D-Zahlensystem, das erstaunlich aehnlich zu den komplexen Zahlen ist.
    #Es hat mich mehr als eine Woche gekostet das ausreichend zu verstehen. Wenn dich der Grund fuer den Code interessiert: einfach melden.
    #Im Allgemeinen wird hier jedoch eine gegebene Koordinate nur nach einer Rotationsanweisung um den Kooridnatenursprung gedreht.
    def qv_mult(self, q1, v1):
        #v1 = tf.transformations.unit_vector(v1) #does not change the vector since its already a unit vector
        q2 = list(v1)
        q2.append(0.0)
        return tf.transformations.quaternion_multiply(
            tf.transformations.quaternion_multiply(q1, q2),
            tf.transformations.quaternion_conjugate(q1)
        )[:3]
    
    #Hie wird eine gegebene Koordinate in Bezug zur Kamera so gestreckt, dass diese nun auf dem Boden liegt. Diese neue Kooridnate wird zurueckgegeben.
    def ground_coordinate(self, cam_coordinate, transform):
        #Hier wird der Streckungs-/Stauchungsfaktor der Koordinate bestimmt, damit diees auf dem Boden liegt.
        factor = (0-transform[2])/cam_coordinate[2]
        #Die entsprechend angepasste Koordinate wird zurueckgegeben.
        return cam_coordinate*factor

    #Hier wird eine gegebene Koordinate um gegebene xyz-Werte einfach verschoben. Dies dient der translatorischen Umwandlung zwischen verschiedenen Frames.
    def transform_coordinate(self, coordinate, transform):
        return coordinate+transform

def main(args=None):
    print('main')
    rclpy.init(args=args)

    pointCloud2FromCamera = PointCloud2FromCamera()

    rclpy.spin(pointCloud2FromCamera)

    pointCloud2FromCamera.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()