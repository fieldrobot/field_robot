## normal cmake list actions
cmake_minimum_required(VERSION 3.5)
project(field_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
## end normal cmake list actions

# finding dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robot_localization REQUIRED)
find_package(field_robot REQUIRED)

############### DIRECTORIES ###############

#installing directories
install(DIRECTORY
  launch
  models
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)

## custom ros communication formats
#settings message files
set(msg_files

)

#settings service files
set(srv_files
)

#setting action files
set(action_files
  "action/ConditionAction.action"
)

#adding defined message, service, and action files to the ros system
rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
#  ${srv_files}
  ${action_files}
)

############### CPP ###############

## POINT CLOUD GENERATOR
add_executable(point_cloud_generator
  src/point_cloud_generator.cpp)

ament_target_dependencies(point_cloud_generator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  PCL
  cv_bridge)

## POINT CLOUD FUSION
add_executable(point_cloud_fusion
  src/point_cloud_fusion.cpp)

ament_target_dependencies(point_cloud_fusion
  rclcpp
  PCL
  geometry_msgs
  sensor_msgs)

## NAVIGATION_BT
add_executable(navigation_bt
  src/navigation/navigation_bt.cpp)

ament_target_dependencies(navigation_bt
  rclcpp
  behaviortree_cpp_v3)

## including cpp "include" directory for cpp header files definitions
include_directories(include)

# installing cpp executables
install(TARGETS
  point_cloud_generator
  point_cloud_fusion
  navigation_bt
  DESTINATION lib/${PROJECT_NAME}
)

############### PYTHON ###############

## installing python modules
ament_python_install_package(${PROJECT_NAME})

## exporting custom ROS communication formats
ament_export_dependencies(rosidl_default_runtime)

## installing python executables
install(PROGRAMS
  scripts/robot_spawner.py
  scripts/demo_image_publisher.py
  scripts/image_path_finder_ai.py
  DESTINATION lib/${PROJECT_NAME}
)

############### BT PLUGINS ###############

# adding (BT) plugins
add_library(continuous_sequence SHARED src/navigation/bt_plugins/control/continuous_sequency_node.cpp)
target_compile_definitions(continuous_sequence PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(continuous_sequence
  behaviortree_cpp_v3)
list(APPEND plugin_libs continuous_sequence)

add_library(finished_headland_drive SHARED src/navigation/bt_plugins/condition/finished_headland_drive_node.cpp)
target_compile_definitions(finished_headland_drive PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(finished_headland_drive
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action)
list(APPEND plugin_libs finished_headland_drive)

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

rosidl_target_interfaces(finished_headland_drive
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

############### ADDITIONAL FILES ###############

## adding RViz Files
install(PROGRAMS
  default.rviz
  DESTINATION share/${PROJECT_NAME}/
)

############### STUFF ###############

## normal/unmodified cmake list actions
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
