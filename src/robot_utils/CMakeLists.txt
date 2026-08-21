cmake_minimum_required(VERSION 3.20)
project(robot_utils)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(robot_utils INTERFACE)
target_include_directories(robot_utils INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")

install(DIRECTORY include/
  DESTINATION include)
install(TARGETS robot_utils
  EXPORT export_robot_utils)

add_executable(system_monitor_node src/system_monitor_node.cpp)
ament_target_dependencies(system_monitor_node
  rclcpp diagnostic_msgs diagnostic_updater nav_msgs robot_interfaces sensor_msgs std_msgs)

add_executable(fault_supervisor_node src/fault_supervisor_node.cpp)
ament_target_dependencies(fault_supervisor_node
  rclcpp robot_interfaces std_srvs)

install(TARGETS system_monitor_node fault_supervisor_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fault_supervisor.launch.py TIMEOUT 30)
endif()

ament_export_targets(export_robot_utils HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp geometry_msgs diagnostic_msgs diagnostic_updater nav_msgs robot_interfaces sensor_msgs std_msgs std_srvs)
ament_package()
