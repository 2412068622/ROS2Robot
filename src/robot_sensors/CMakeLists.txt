cmake_minimum_required(VERSION 3.20)
project(robot_sensors)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(laser_filter_node src/laser_filter_node.cpp)
ament_target_dependencies(laser_filter_node rclcpp sensor_msgs diagnostic_msgs diagnostic_updater)

add_executable(imu_filter_node src/imu_filter_node.cpp)
ament_target_dependencies(imu_filter_node rclcpp sensor_msgs diagnostic_msgs diagnostic_updater)

add_executable(fake_scan_node src/fake_scan_node.cpp)
ament_target_dependencies(fake_scan_node rclcpp sensor_msgs)

add_executable(imu_100d4_parser_node src/imu_100d4_parser_node.cpp)
ament_target_dependencies(imu_100d4_parser_node rclcpp sensor_msgs std_msgs std_srvs)

install(TARGETS laser_filter_node imu_filter_node fake_scan_node imu_100d4_parser_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
