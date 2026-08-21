cmake_minimum_required(VERSION 3.20)
project(robot_hardware)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_interfaces_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(robot_hardware_backends
  src/chassis_backend_factory.cpp
  src/chassis_kinematics.cpp
  src/chassis_packet.cpp
  src/chassis_system_adapter.cpp
  src/mock_backend.cpp
  src/serial_backend.cpp
  src/udp_backend.cpp)
target_include_directories(robot_hardware_backends PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(robot_hardware_backends PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(robot_chassis_hardware SHARED
  src/chassis_hardware_interface.cpp)
target_link_libraries(robot_chassis_hardware robot_hardware_backends)
ament_target_dependencies(robot_chassis_hardware
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface robot_hardware_plugins.xml)

add_executable(chassis_driver_node src/chassis_driver_node.cpp)
target_link_libraries(chassis_driver_node robot_hardware_backends)
ament_target_dependencies(chassis_driver_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_srvs tf2_ros robot_interfaces robot_interfaces_core)

add_executable(chassis_simulator_node src/chassis_simulator_node.cpp)
target_link_libraries(chassis_simulator_node robot_hardware_backends)
ament_target_dependencies(chassis_simulator_node rclcpp)

add_executable(odom_path_recorder_node src/odom_path_recorder_node.cpp)
ament_target_dependencies(odom_path_recorder_node rclcpp nav_msgs std_srvs)

install(TARGETS robot_hardware_backends robot_chassis_hardware
  EXPORT export_robot_hardware_backends
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS chassis_driver_node chassis_simulator_node odom_path_recorder_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES robot_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(chassis_packet_test test/chassis_packet_test.cpp)
  target_link_libraries(chassis_packet_test robot_hardware_backends)
  ament_add_gtest(chassis_system_adapter_test test/chassis_system_adapter_test.cpp)
  target_link_libraries(chassis_system_adapter_test robot_hardware_backends)
  ament_add_gtest(
    chassis_hardware_interface_test
    test/chassis_hardware_interface_test.cpp)
  target_link_libraries(chassis_hardware_interface_test robot_chassis_hardware)
  ament_target_dependencies(chassis_hardware_interface_test
    hardware_interface rclcpp rclcpp_lifecycle)
endif()

ament_export_include_directories(include)
ament_export_targets(export_robot_hardware_backends HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib)
ament_package()
