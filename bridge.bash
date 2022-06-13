source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
export ROS_MASTER_URI=http://172.20.0.5:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics