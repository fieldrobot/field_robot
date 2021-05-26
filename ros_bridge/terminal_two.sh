. /opt/ros/noetic/setup.bash
. /opt/ros/foxy/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics