#!/usr/bin/env python3
import cv2
import numpy as np
import random
from geometry_msgs.msg import Pose, Point, Quaternion

from threading import Thread

import time

import subprocess
import os

def threaded_function(arg):
    subprocess.call(['sh', './field_robot/src/virtual_maize_field/virtual_maize_field/start.sh'])

def main() -> None:

    while (True):
        thread = Thread(target = threaded_function, args = (10, ))
        thread.start()
        time.sleep(30)
        os.system("ros2 run virtual_maize_field robot_spawner")
        time.sleep(10)

        #record images

        os.system("ros2 run virtual_maize_field robot_delete")
        time.sleep(30)
        os.system("pkill -9 gzserver && pkill -9 gzclient")

if __name__ == "__main__":
    main()
