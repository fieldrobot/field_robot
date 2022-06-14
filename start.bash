#!/bin/bash
ls /field_robot/ROS1/devel
echo start_bash
bash /field_robot/field_robot/robot.bash &
bash /field_robot/field_robot/ros1rob.bash &
bash /field_robot/field_robot/bridge.bash && fg

sleep 99999