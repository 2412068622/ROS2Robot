cmake_minimum_required(VERSION 3.20)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_mock_bringup.launch.py TIMEOUT 30)
endif()

ament_package()
