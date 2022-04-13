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
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robot_localization REQUIRED)

#installing launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

#installing directories
install(DIRECTORY
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
  #"service/BTNode.srv"
)

#setting action files
set(action_files
  #"action/BTNode.action"
)

#adding defined message, service, and action files to the ros system
#rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
#  ${srv_files}
#  ${action_files}
#)

## including cpp "include" directory for cpp header files definitions
include_directories(include)

# CPP EXECUTABLES
## POINT CLOUD GENERATOR
add_executable(point_cloud_generator
  src/point_cloud_generator.cpp)

ament_target_dependencies(point_cloud_generator
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  OpenCV
  cv_bridge)

add_executable(point_cloud_fusion
  src/point_cloud_fusion.cpp)

ament_target_dependencies(point_cloud_fusion
  rclcpp
  sensor_msgs)

#rosidl_target_interfaces(navigation
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# installing cpp executables
install(TARGETS
  point_cloud_generator
  point_cloud_fusion
  DESTINATION lib/${PROJECT_NAME}
)

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
