cmake_minimum_required(VERSION 3.20)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateSequence.action"
  "msg/TrackingError.msg"
  "msg/TaskStatus.msg"
  "msg/RobotState.msg"
  "msg/ChassisState.msg"
  "msg/NavigationStatistics.msg"
  "msg/MapZone.msg"
  "msg/PayloadState.msg"
  "msg/LocalizationHealth.msg"
  "msg/SafetyState.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
