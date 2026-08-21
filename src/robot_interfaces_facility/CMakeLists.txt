cmake_minimum_required(VERSION 3.20)
project(robot_interfaces_facility)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PayloadState.msg"
  "srv/ListFacilityResources.srv"
  "srv/ReserveFacilityResource.srv"
  "srv/ReleaseFacilityResource.srv"
  "srv/ExecuteFacilityAction.srv"
  "srv/RequestLiftSession.srv"
  "srv/ReleaseLiftSession.srv"
  "srv/GetLiftState.srv"
  "srv/GetDoorState.srv"
  "srv/ConfirmLoad.srv"
  "srv/ConfirmUnload.srv"
  "srv/SetPayloadLoaded.srv"
  "srv/ExecuteTopModuleAction.srv"
  "srv/RequestStationConfirmation.srv"
  "srv/GetPendingConfirmations.srv"
  "srv/ConfirmStationAction.srv"
  "srv/RejectStationAction.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
