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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

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
  "action/ComputeGoalAction.action"
  "action/TurnInformationAction.action"
  "action/ComputeBasedGoalAction.action"
  "action/RobotPoseUpdateAction.action"
)

#adding defined message, service, and action files to the ros system
rosidl_generate_interfaces(${PROJECT_NAME}
#  ${msg_files}
#  ${srv_files}
  ${action_files}
  DEPENDENCIES geometry_msgs
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

rosidl_target_interfaces(point_cloud_generator
${PROJECT_NAME} "rosidl_typesupport_cpp")

## POINT CLOUD FUSION
add_executable(point_cloud_fusion
  src/point_cloud_fusion.cpp)

ament_target_dependencies(point_cloud_fusion
  rclcpp
  PCL
  geometry_msgs
  sensor_msgs)

rosidl_target_interfaces(point_cloud_fusion
${PROJECT_NAME} "rosidl_typesupport_cpp")

## NAVIGATION_BT
add_executable(navigation_bt
  src/navigation/navigation_bt.cpp)

ament_target_dependencies(navigation_bt
  rclcpp
  behaviortree_cpp_v3)

rosidl_target_interfaces(navigation_bt
${PROJECT_NAME} "rosidl_typesupport_cpp")

## FRONT EMPTY ACTION SERVER
add_executable(front_empty_server
  src/navigation/action_servers/condition/front_empty_node_server.cpp)

ament_target_dependencies(front_empty_server
  rclcpp
  rclcpp_action
  sensor_msgs)

rosidl_target_interfaces(front_empty_server
${PROJECT_NAME} "rosidl_typesupport_cpp")

## COMPLETED LEAVING ROW ACTION SERVER
add_executable(completed_leaving_row_server
  src/navigation/action_servers/condition/completed_leaving_row_turn_action.cpp)

ament_target_dependencies(completed_leaving_row_server
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

rosidl_target_interfaces(completed_leaving_row_server
${PROJECT_NAME} "rosidl_typesupport_cpp")

## PUBLISH TURN INFORMATION ACTION SERVER
add_executable(publish_turn_information_server
  src/navigation/action_servers/action/publish_turn_information_server.cpp)

ament_target_dependencies(publish_turn_information_server
  rclcpp
  rclcpp_action)

rosidl_target_interfaces(publish_turn_information_server
${PROJECT_NAME} "rosidl_typesupport_cpp")

## NEW GOAL COMPUTABLE ACTION SERVER
add_executable(new_goal_computable_server
  src/navigation/action_servers/condition/new_goal_computable_headland_drive_server.cpp)

ament_target_dependencies(new_goal_computable_server
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  PCL)

rosidl_target_interfaces(new_goal_computable_server
${PROJECT_NAME} "rosidl_typesupport_cpp")

## COMPUTE GOAL LEAVING ROW ACTION SERVER
add_executable(compute_goal_leaving_row_server
  src/navigation/action_servers/action/compute_goal_leaving_row_server.cpp)

ament_target_dependencies(compute_goal_leaving_row_server
  rclcpp
  rclcpp_action
  tf2_ros
  tf2_geometry_msgs)

rosidl_target_interfaces(compute_goal_leaving_row_server
${PROJECT_NAME} "rosidl_typesupport_cpp")

## COMPUTE GOAL IN ROW ACTION SERVER
add_executable(compute_goal_in_row_server
  src/navigation/action_servers/action/compute_goal_in_row_server.cpp)

ament_target_dependencies(compute_goal_in_row_server
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  PCL)

rosidl_target_interfaces(compute_goal_in_row_server
${PROJECT_NAME} "rosidl_typesupport_cpp")

## COMPUTE GOAL HEADLAND DRIVE ACTION SERVER
add_executable(compute_goal_headland_drive_server
  src/navigation/action_servers/action/compute_goal_headland_drive_server.cpp)

ament_target_dependencies(compute_goal_headland_drive_server
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  PCL)

rosidl_target_interfaces(compute_goal_headland_drive_server
${PROJECT_NAME} "rosidl_typesupport_cpp")

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
rosidl_target_interfaces(continuous_sequence
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(simple_check_nodes SHARED src/navigation/bt_plugins/condition/simple_check_nodes.cpp)
target_compile_definitions(simple_check_nodes PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(simple_check_nodes
  behaviortree_cpp_v3
  nav2_behavior_tree)
list(APPEND plugin_libs simple_check_nodes)
rosidl_target_interfaces(simple_check_nodes
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(compute_goals SHARED src/navigation/bt_plugins/action/compute_goals.cpp)
target_compile_definitions(compute_goals PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(compute_goals
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs)
list(APPEND plugin_libs compute_goals)
rosidl_target_interfaces(compute_goals
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(compute_based_goals SHARED src/navigation/bt_plugins/action/compute_based_goals.cpp)
target_compile_definitions(compute_based_goals PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(compute_based_goals
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs)
list(APPEND plugin_libs compute_based_goals)
rosidl_target_interfaces(compute_based_goals
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(new_goal_computable SHARED src/navigation/bt_plugins/condition/new_goal_computable_headland_drive_node.cpp)
target_compile_definitions(new_goal_computable PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(new_goal_computable
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs)
list(APPEND plugin_libs new_goal_computable)
rosidl_target_interfaces(new_goal_computable
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(store_turn_information SHARED src/navigation/bt_plugins/action/store_turn_information_node.cpp)
target_compile_definitions(store_turn_information PRIVATE  BT_PLUGIN_EXPORT)
ament_target_dependencies(store_turn_information
  behaviortree_cpp_v3
  nav2_behavior_tree
  geometry_msgs)
list(APPEND plugin_libs store_turn_information)
rosidl_target_interfaces(store_turn_information
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
