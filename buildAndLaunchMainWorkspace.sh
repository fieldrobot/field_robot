#1/bin/bash
clear
workspace="dev_ws/"
echo Workspace: $workspace
echo Building
cd dev_ws
colcon build
ros2 launch field_robot simulation.launch.py
