echo robot_start
source /opt/ros/foxy/setup.bash
cd /field_robot/field_robot/dev_ws && colcon build
source /field_robot/field_robot/dev_ws/install/setup.bash
ros2 launch field_robot navigation.launch.py &
ros2 launch field_robot operating_services.launch.py && fg