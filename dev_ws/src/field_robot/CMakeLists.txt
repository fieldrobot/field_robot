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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
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
  robot_description
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)

#adding action files
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Emptyspacefollower.action"
  "action/HeadlandTurn.action"
  DEPENDENCIES
    builtin_interfaces
)

#include cpp "include" directories
include_directories(include)

#create cpp executables
#add_executable(navigation src/navigation.cpp)
add_library(navigation SHARED
  src/navigation.cpp)
target_include_directories(navigation PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(navigation
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(navigation
  rclcpp
  rclcpp_action
  behaviortree_cpp_v3)
#rclcpp_components_register_node(navigation PLUGIN "field_robot::FibonacciActionClient" EXECUTABLE fibonacci_action_client)
install(TARGETS
    navigation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

#install cpp executables
install(TARGETS
  navigation
  DESTINATION lib/${PROJECT_NAME}
)

#install python modules
ament_python_install_package(${PROJECT_NAME})

#install python executables
install(PROGRAMS
  scripts/robot_spawner.py
  scripts/empty_space_follower_two.py
  scripts/headland_turn.py
  DESTINATION lib/${PROJECT_NAME}
)

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
