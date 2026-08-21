cmake_minimum_required(VERSION 3.20)
project(robot_experiments)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(experiment_runner_node src/experiment_runner_node.cpp)
ament_target_dependencies(experiment_runner_node
  rclcpp nav_msgs geometry_msgs std_msgs visualization_msgs robot_interfaces)

install(TARGETS experiment_runner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY paths results
  DESTINATION share/${PROJECT_NAME})

ament_package()
