#!/usr/bin/env python3
import cv2
import numpy as np
import random
from geometry_msgs.msg import Pose, Point, Quaternion
import dis
import math
import random
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

from threading import Thread

import time

import subprocess
import os

# Robot Variables
robotRadius = .25

# Spawnpoint Variables
pointAmounts = [1,1,1] # Field, Padding, Edge
spawnHeight = 1
minDistance = robotRadius*2
maxDistance = robotRadius*3

# World Variables
worldFilePath = get_package_share_directory('virtual_maize_field') + '/worlds/generated.world'
plantSpacing = .75
field_padding = 1
field_edge = -2

def threaded_function(arg):
    #subprocess.call(['sh', '/field_robot/src/virtual_maize_field/virtual_maize_field/start.sh'])
    subprocess.call(['bash', get_package_share_directory('virtual_maize_field') + '/virtual_maize_field/start.bash'])

def main() -> None:

    i = 1

    while (True):
        thread = Thread(target = threaded_function, args = (10, ))
        thread.start()
        time.sleep(30)

        points = generateCoordinates()

        i = 1

        for point in points:
            name = 'robot_' + str(i)
            st = f'ros2 run virtual_maize_field robot_spawner --ros-args -p "x:={str(point[0])}F" -p "y:={str(point[1])}F" -p "z:={str(point[2])}F" -p "name:={name}"'
            print(st)
            #os.system(st)
            time.sleep(5)
            print('asdfjalsdfkjasöldfkjasdölfkj')
            #ros2 run virtual_maize_field robot_spawner --ros-args -p "x:=str(point[0])" -p "y:=str(point[1])" -p z:="str(point[2])
            #os.system("ros2 run virtual_maize_field save_img")
            time.sleep(5)
            print('w48czw34zwme8uxmw4u9w,4pxw0u,4tp90u4')
            #record images
            st2 = f'ros2 run virtual_maize_field robot_delete --ros-args -p "name:={name}"'
            #os.system(st2)
            time.sleep(5)
            i = i + 1
        #thread.kill()
        os.system("pkill -9 gzserver && pkill -9 gzclient")

def generateCoordinates():

    def calcDistance(start, goal):
        value = 0
        for i in range(2):
            value += (start[i] - goal[i]) ** 2
        return math.sqrt(value)

    # Returns the minimum Distance to a plant, allthough Distances of 0 are ignored
    def calcMinDistance(pos):
        dist = -1
        for plant in plants:
            tempDist = calcDistance(pos, plant)
            if (dist == -1 or tempDist < dist) and tempDist > 0:
                dist = tempDist
        return dist
        
    # Load File
    print("Loading World")
    root = ET.parse(worldFilePath).getroot()
    print(f"Found root: <{root.tag}/>")

    plants = []

    for plantPose in root.findall('world/include/pose'):
        pos = plantPose.text.split(' ')[:3]
        for spawnPoint in range(3):
            pos[spawnPoint] = float(pos[spawnPoint])
        plants.append(pos)

    min_cord = plants[0].copy()
    max_cord = plants[0].copy()

    for plantPose in plants:
        for spawnPoint in range(3):
            min_cord[spawnPoint] = min(min_cord[spawnPoint], plantPose[spawnPoint])
            max_cord[spawnPoint] = max(max_cord[spawnPoint], plantPose[spawnPoint])

    print(f'X min: {min_cord[0]},\tmax: {max_cord[0]}')
    print(f'Y min: {min_cord[1]},\tmax: {max_cord[1]}')
    print(f'Z min: {min_cord[2]},\tmax: {max_cord[2]}')

    print()

    spawnpoints = []

    def randomPointInField(dif):
        pos1 = plants[random.randint(0,len(plants)-1)]
        pos2 = plants[random.randint(0,len(plants)-1)]
        return [
            (pos1[0] - pos2[0])/2+pos2[0],
            (pos1[1] - pos2[1])/2+pos2[1],
            spawnHeight
        ]

    def randomPointInRange(xMin, xMax, yMin, yMax):
        return [
            random.uniform(xMin, xMax),
            random.uniform(yMin, yMax),
            spawnHeight
        ]

    def randomPointOnEdge(dif):
        randInt=random.randint(0,3)
        if randInt == 0:
            return randomPointInRange(
                min_cord[0]-dif, min_cord[0],
                min_cord[1], max_cord[1]
            )
        elif randInt == 1:
            return randomPointInRange(
                min_cord[0], max_cord[0],
                min_cord[1]-dif, min_cord[1]
            )
        elif randInt == 2:
            return randomPointInRange(
                max_cord[0], max_cord[0]+dif,
                min_cord[1], max_cord[1]
            )
        elif randInt == 3:
            return randomPointInRange(
                min_cord[0], max_cord[0],
                max_cord[1], max_cord[1]+dif
            )

    def generateSpawnpointCandidate(point_type):
        # Random
        if point_type == 0:
            return randomPointInField(0)

        # Padding
        elif point_type ==  1:
            return randomPointOnEdge(field_padding)

        # Edge
        elif point_type == 2:
            return randomPointOnEdge(-field_edge)
        
        # default
        else :
            print("Type not recognized, generating Random Point")
            return randomPointInField(0)

    print("Generating Random Points")
    for point_type in range(len(pointAmounts)):
        for spawnPoint in range(pointAmounts[point_type]):
            pos = generateSpawnpointCandidate(point_type)
            distance = calcMinDistance(pos)
            while distance < minDistance or distance > maxDistance:
                pos = generateSpawnpointCandidate(point_type)
                distance = calcMinDistance(pos)
            spawnpoints.append(pos)

    for spawnPoint in spawnpoints:
        print(f'Min. distance: {"{:.2f}".format(calcMinDistance(spawnPoint))}m, Pos: {spawnPoint}')

    return spawnpoints

if __name__ == "__main__":
    main()
