cmake_minimum_required(VERSION 3.20)
project(robot_path_tracking)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

include_directories(include)

add_executable(path_publisher_node src/path_publisher_node.cpp)
ament_target_dependencies(path_publisher_node rclcpp geometry_msgs nav_msgs)

add_executable(pure_pursuit_controller_node src/pure_pursuit_controller_node.cpp)
ament_target_dependencies(pure_pursuit_controller_node
  rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs robot_interfaces)

add_executable(stanley_controller_node src/stanley_controller_node.cpp)
ament_target_dependencies(stanley_controller_node
  rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs robot_interfaces)

install(TARGETS
  path_publisher_node
  pure_pursuit_controller_node
  stanley_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tracking_math_test test/tracking_math_test.cpp)
  target_include_directories(tracking_math_test PRIVATE include)
endif()

ament_export_include_directories(include)
ament_package()
