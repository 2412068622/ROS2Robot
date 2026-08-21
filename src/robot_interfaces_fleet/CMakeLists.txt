cmake_minimum_required(VERSION 3.20)
project(robot_interfaces_fleet)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ListFleetRobots.srv"
  "srv/UpdateFleetRobotState.srv"
  "srv/UpdateRemoteTaskState.srv"
  "srv/PreviewFleetStationTask.srv"
  "srv/SubmitFleetStationTask.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
