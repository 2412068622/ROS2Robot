cmake_minimum_required(VERSION 3.20)
project(robot_teleop)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_interfaces_core REQUIRED)

include_directories(include)

add_executable(cmd_vel_mux_node src/cmd_vel_mux_node.cpp)
ament_target_dependencies(cmd_vel_mux_node
  rclcpp geometry_msgs std_msgs std_srvs robot_interfaces robot_interfaces_core)

add_executable(cmd_vel_safety_gate_node src/cmd_vel_safety_gate_node.cpp)
ament_target_dependencies(cmd_vel_safety_gate_node
  rclcpp geometry_msgs std_msgs std_srvs robot_interfaces robot_interfaces_core)

add_executable(virtual_rc_node src/virtual_rc_node.cpp)
ament_target_dependencies(virtual_rc_node
  rclcpp geometry_msgs sensor_msgs std_msgs std_srvs)

add_executable(keyboard_teleop_node src/keyboard_teleop_node.cpp)
ament_target_dependencies(keyboard_teleop_node
  rclcpp geometry_msgs)

install(TARGETS cmd_vel_mux_node cmd_vel_safety_gate_node virtual_rc_node keyboard_teleop_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(cmd_vel_safety_test test/cmd_vel_safety_test.cpp)
  target_include_directories(cmd_vel_safety_test PRIVATE include)
  ament_target_dependencies(cmd_vel_safety_test geometry_msgs)
endif()

ament_export_include_directories(include)
ament_package()
