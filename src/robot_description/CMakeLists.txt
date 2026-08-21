cmake_minimum_required(VERSION 3.20)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config urdf rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
