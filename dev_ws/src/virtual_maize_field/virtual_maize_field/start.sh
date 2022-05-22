#export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
source /usr/share/gazebo-11/setup.bash
ros2 run virtual_maize_field generate_world
ros2 launch virtual_maize_field simulation.launch.py
