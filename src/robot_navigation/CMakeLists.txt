cmake_minimum_required(VERSION 3.20)
project(robot_navigation)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(robot_interfaces_navigation REQUIRED)
find_package(std_msgs REQUIRED)

add_library(robot_navigation_map_catalog
  src/map_catalog.cpp)
target_include_directories(robot_navigation_map_catalog PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(robot_navigation_zone_catalog
  src/zone_catalog.cpp)
target_include_directories(robot_navigation_zone_catalog PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(robot_navigation_zone_filter_masks
  src/zone_filter_masks.cpp)
target_include_directories(robot_navigation_zone_filter_masks PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(robot_navigation_zone_filter_masks
  robot_navigation_zone_catalog)

add_executable(map_manager_node src/map_manager_node.cpp)
target_link_libraries(map_manager_node
  robot_navigation_map_catalog
  robot_navigation_zone_catalog)
ament_target_dependencies(map_manager_node
  ament_index_cpp rclcpp robot_interfaces robot_interfaces_navigation std_msgs)

add_executable(export_zone_filter_masks src/export_zone_filter_masks.cpp)
target_link_libraries(export_zone_filter_masks
  robot_navigation_zone_catalog
  robot_navigation_zone_filter_masks)

install(DIRECTORY launch config maps
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS
    robot_navigation_map_catalog
    robot_navigation_zone_catalog
    robot_navigation_zone_filter_masks
  EXPORT export_robot_navigation_map_catalog
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS map_manager_node export_zone_filter_masks
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(map_catalog_test test/map_catalog_test.cpp)
  target_link_libraries(map_catalog_test robot_navigation_map_catalog)
  ament_add_gtest(zone_catalog_test test/zone_catalog_test.cpp)
  target_link_libraries(zone_catalog_test robot_navigation_zone_catalog)
  ament_add_gtest(zone_filter_masks_test test/zone_filter_masks_test.cpp)
  target_link_libraries(zone_filter_masks_test robot_navigation_zone_filter_masks)
  add_launch_test(test/test_slam_mock_runtime.launch.py TIMEOUT 45)
  add_launch_test(test/test_nav2_mock_runtime.launch.py TIMEOUT 150)
endif()

ament_export_include_directories(include)
ament_export_targets(export_robot_navigation_map_catalog HAS_LIBRARY_TARGET)
ament_package()
