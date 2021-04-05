# importing libraries used for computation
import cv2
import cv_bridge
import numpy

# necessary to make parallel processes sleeping so that they do not overwork
import time

# importing the client library for python: default for every node
import rclpy
# importing the node, necessary to register this script as a node
from rclpy.node import Node
# importing libraries to keep the aciton server running
from rclpy.action import ActionServer, CancelResponse # default action server and making canceling possible
# necessary to execute callbacks at the same time without one blocking the other ones
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# quality of service profily needs to be created as the image is published not with default configuration
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile

# importing message and action files to receive and send data via ROS2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from field_robot.action import EmptySpaceFollower

# class handling everything concerning empty space following
class EmptySpaceFollowerServer(Node):

    # constructor for the empty space follower
    def __init__(self):
        #starting node and naming it
        super().__init__('empty_space_follower')

        self.get_logger().info('starting...')
        
        # creating the cv birdge for image processing
        self.bridge = cv_bridge.CvBridge()

        # this variable is refered to by the image callback and action server and represents the state of the system
        # true: navigaiton is working and cmd_vel is published
        # false: navigation is halted and no velocity commands are published, some other node is responsible for publishing those
        self.working = False

        # setting up the publisher for velocity commands
        self.publish = self.create_publisher(Twist, '/cmd_vel', 10)

        # registering the action server, defining its callbacks and declaring it's callback group (necessary for parallel execution)
        self.action_server = ActionServer(
            node=self, # the node (this class) to associate with
            action_type=EmptySpaceFollower, # action message type
            action_name='empty_space_follower', # action name
            execute_callback=self.action_callback, # the callback called when action server request accepted (by default: all are accepted)
            callback_group=ReentrantCallbackGroup(), # making parallel execution possible by multithreading
            cancel_callback=self.cancel_callback # the cancel callback makes is possible to stop the navigation
        )

        # declaring the qos_profile for image subscription
        self._qos_profile = QoSProfile(
            history=QoSHistoryPolicy.SYSTEM_DEFAULT
        )
        self._qos_profile.history = QoSHistoryPolicy.SYSTEM_DEFAULT
        self._qos_profile.depth = 10
        self._qos_profile.durability = QoSDurabilityPolicy.SYSTEM_DEFAULT
        self._qos_profile.reliability = QoSReliabilityPolicy.SYSTEM_DEFAULT
        # subscribing the image and declaring callbacks and the qos profile
        self.subscription = self.create_subscription(
            msg_type=Image, # the message file the received data has
            topic='field_robot/gazebo_cam/image_raw', # the topic to be subscribed
            callback=self.image_callback, # declaring the callback for image processing 
            callback_group=ReentrantCallbackGroup(), # making parallel execution possible my multithreading
            qos_profile=self._qos_profile # the qos profile
        )

        self.get_logger().info('initialized')

    # called when action server request ist accepted -> implementing the actual action
    def action_callback(self, goal_handle):
        self.get_logger().info('Setting goal...')
        # setting the working variable to true so that the image processing starts and cmd_vel is published
        self.working = True
        self.get_logger().info('Successfully initiated navigation')
        # returning the result of the action message file is mandatory
        result = EmptySpaceFollower.Result()
        # checks constantly whether there there is a request that the execution is canceled and navigation is stopped
        while True:
            time.sleep(0.2) # to reduce computational needs: pause
            # is executed if navigaiton is to be stopped/cancelled
            if goal_handle.is_cancel_requested:
                # setting the working variable to that no cmd_vel commands are published any more
                self.working = False
                time.sleep(0.2)

                # publishing 0 values for all aspects so that the robot stops
                twist_msg = Twist()
                self.publish.publish(twist_msg)

                # confirming cancel
                goal_handle.canceled()
                return result
        # the action can not return successful as this script is just navigation continuously

    # declaring in the callback called when navigation is to be stopped that all requests to do so are accepted
    def cancel_callback(self, cancel_requests):
        return CancelResponse.ACCEPT

    # the image processing/actual navigation
    def image_callback(self, msg):
        self.get_logger().info('Calling image callback...')
        if self.working == False:
            self.get_logger().info('Inactive')
            return
        self.get_logger().info('Starting image processing...')

        #### image preparation

        # convertin image to opencv2
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, d = image.shape

        # convertin cv2 image to hsv
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # filtering image for green corn
        lower_green = numpy.array([30, 100, 0])
        upper_green = numpy.array([80, 255, 255])
        image_filtered = cv2.inRange(image_hsv, lower_green, upper_green)

        # dilating image: making everything more bold -> flatten
        image_dilated = cv2.dilate(image_filtered, None, iterations=8)
        
        #### searching the biggenst stripe without any plante
        streifen = 50
        max = w
        min = 0

        while (max-min) > 2:
            if self.mindestens_einmal(streifen, image_dilated):
                min = streifen
                streifen = streifen + ((max - min)/2)
            else:
                max = streifen
                streifen = streifen - ((max - min)/2)

        #### calculating the x axis middle of the biggest stripe
        sm = self.streifen_mitte(streifen, image_dilated)

        #### visualizing that middle
        image_with_point = cv2.circle(image_dilated, (int(sm), 50), 20, (255), -20)
        #cv2.imshow("kevin", image_with_point)
        
        #### regulating the velocity command
        #calculation the error
        self.error = (w/2)-sm #negative: to far to the left -> right &&&&&&& positive: to far to the right -> left

        #preparing the message to be send
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        #targets/configuaration
        # unused so far: linear_target = 0.2
        angular_d = 0.002
        # unused so far: linear_d = 1

        #p-control
        twist_msg.angular.z = self.error * angular_d
        #twist_msg.linear.x = linear_target + linear_d / error

        # checking whether publishing is still acceptable
        if self.working == False:
            self.get_logger().info('Stopped publishing due to canceled action.')
            return

        #publishing the velocity commands
        self.publish.publish(twist_msg)

        #cv2.waitKey(1)

    def mindestens_einmal(self, streifen, image):
        count = 0
        array = []
        image_t = image.T
        #cv2.imshow("oho",image_t)
        #cv2.imshow("oha",image)
        for column in image_t:
            if not numpy.any(column):
                count = count+1
                #print('0')
            elif count != 0:
                if count >= streifen:
                    array.append(count)
                count = 0
                #print('1')
            #else:
                #print('1')
        if count != 0 and count >= streifen: array.append(count)
        if len(array) >= 1: return True
        return False

    def streifen_mitte(self, streifen, image):
        streifen = streifen - 10
        iterator = 0
        start = 0
        image_t = image.T
        for column in image_t:
            if numpy.any(column) and (iterator-start) > streifen:
                return (iterator + start)/2
            elif numpy.any(column):
                start = iterator
            iterator = iterator+1 
        if (iterator-start) > streifen: return (iterator + start)/2
        print('fuck')
        return iterator/2

def main():
    rclpy.init()
    follower = EmptySpaceFollowerServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(follower, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()