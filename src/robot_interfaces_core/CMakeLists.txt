cmake_minimum_required(VERSION 3.20)
project(robot_interfaces_core)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/TaskStatus.msg"
  "msg/ChassisState.msg"
  "msg/NavigationStatistics.msg"
  "msg/TrackingError.msg"
  "srv/SetControlMode.srv"
  "srv/SetChassisMode.srv"
  "srv/GetStatistics.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
