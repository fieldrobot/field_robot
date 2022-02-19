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
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

#installing launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

#installing directories
install(DIRECTORY
  models
  config
  DESTINATION share/${PROJECT_NAME}/
)

## custom ros communication formats
#settings message files
set(msg_files

)

#settings service files
set(srv_files
  "service/BTNode.srv"
)

#setting action files
set(action_files
  "action/BTNode.action"
)

#adding defined message, service, and action files to the ros system
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
)

## including cpp "include" directory for cpp header files definitions
include_directories(include)

## creating cpp executables
# creating the behavior_tree executable
add_executable(navigation
  src/navigation.cpp)
ament_target_dependencies(navigation
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3)

rosidl_target_interfaces(navigation
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

## installing cpp executables
install(TARGETS
  navigation
  DESTINATION lib/${PROJECT_NAME}
)

## installing python modules
ament_python_install_package(${PROJECT_NAME})

## exporting custom ROS communication formats
ament_export_dependencies(rosidl_default_runtime)

## installing python executables
install(PROGRAMS
  scripts/robot_spawner.py
  scripts/empty_space_follower_two.py
  scripts/headland_turn.py
  scripts/service_in_row.py
  scripts/wheel_odometry.py
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
