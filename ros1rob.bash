#!/bin/bash
echo spawner_start
source /opt/ros/noetic/setup.bash
cp -r /catkin/src/virtual_maize_field/launch /field_robot/ROS1/src/virtual_maize_field
cp -r /catkin/src/virtual_maize_field/map /field_robot/ROS1/src/virtual_maize_field
cp -r /catkin/src/virtual_maize_field/models /field_robot/ROS1/src/virtual_maize_field
cd /field_robot/ROS1 && catkin_make
source /field_robot/ROS1/devel/setup.bash
export ROS_MASTER_URI=http://simulation:11311
roslaunch example_robot_brain task_navigation.launch --wait --screen