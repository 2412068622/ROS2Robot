cmake_minimum_required(VERSION 3.20)
project(robot_simulation)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_interfaces_business REQUIRED)
find_package(robot_interfaces_facility REQUIRED)
find_package(robot_interfaces_fleet REQUIRED)
find_package(robot_interfaces_mission REQUIRED)
find_package(robot_interfaces_navigation REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(robot_simulation_planar_odometry src/planar_odometry.cpp)
target_include_directories(robot_simulation_planar_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(robot_simulation_planar_odometry nav_msgs tf2)

add_executable(amr_sim_visualizer_node src/amr_sim_visualizer_node.cpp)
ament_target_dependencies(amr_sim_visualizer_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  robot_interfaces_business
  robot_interfaces_facility
  robot_interfaces_fleet
  robot_interfaces_mission
  std_msgs
  visualization_msgs)

add_executable(amr_sim_demo_director_node src/amr_sim_demo_director_node.cpp)
ament_target_dependencies(amr_sim_demo_director_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  robot_interfaces_business
  robot_interfaces_facility
  robot_interfaces_fleet
  robot_interfaces_mission
  robot_interfaces_navigation
  std_msgs)

add_executable(amr_sim_gazebo_entity_bridge_node src/amr_sim_gazebo_entity_bridge_node.cpp)
ament_target_dependencies(amr_sim_gazebo_entity_bridge_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  robot_interfaces_facility
  robot_interfaces_fleet
  robot_interfaces_mission
  ros_gz_interfaces
  std_msgs)

add_executable(robot2_fleet_demo_driver_node src/robot2_fleet_demo_driver_node.cpp)
ament_target_dependencies(robot2_fleet_demo_driver_node
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  robot_interfaces_fleet
  robot_interfaces_mission)

add_executable(gazebo_odom_tf_node src/gazebo_odom_tf_node.cpp)
target_link_libraries(gazebo_odom_tf_node robot_simulation_planar_odometry)
ament_target_dependencies(gazebo_odom_tf_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2_ros)

add_executable(gazebo_unpause_node src/gazebo_unpause_node.cpp)
ament_target_dependencies(gazebo_unpause_node
  rclcpp
  ros_gz_interfaces)

install(TARGETS
  amr_sim_visualizer_node
  amr_sim_demo_director_node
  amr_sim_gazebo_entity_bridge_node
  robot2_fleet_demo_driver_node
  gazebo_odom_tf_node
  gazebo_unpause_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS robot_simulation_planar_odometry
  EXPORT export_robot_simulation_planar_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch media rviz worlds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(planar_odometry_test test/planar_odometry_test.cpp)
  target_link_libraries(planar_odometry_test robot_simulation_planar_odometry)
  ament_target_dependencies(planar_odometry_test nav_msgs tf2)

  ament_add_pytest_test(
    model_geometry_consistency_test
    test/model_geometry_consistency_test.py
    TIMEOUT 90
    APPEND_ENV
      "ROBOT_DESCRIPTION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../robot_description"
      "ROBOT_SIMULATION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
      "ROBOT_NAVIGATION_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../robot_navigation"
      "ROBOT_HARDWARE_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../robot_hardware"
      "ROBOT_BRINGUP_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}/../robot_bringup")

  add_launch_test(test/test_sim_model_runtime.launch.py TIMEOUT 180)
endif()

ament_export_include_directories(include)
ament_export_targets(export_robot_simulation_planar_odometry HAS_LIBRARY_TARGET)
ament_package()
