cmake_minimum_required(VERSION 3.20)
project(robot_interfaces_mission)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/EnqueueMission.srv"
  "srv/PreemptMission.srv"
  "srv/CancelQueuedMission.srv"
  "srv/ReprioritizeQueuedMission.srv"
  "srv/ReprioritizeBatch.srv"
  "srv/GetMissionRecoveryState.srv"
  "srv/ListMissionEvents.srv"
  "srv/EstimateMissionCost.srv"
  "srv/EstimateStationTransport.srv"
  "srv/ListStations.srv"
  "srv/ListStationRoutes.srv"
  "srv/SubmitStationTransportOrder.srv"
  "srv/SubmitStationOrderBatch.srv"
  "srv/CancelStationOrderBatch.srv"
  "srv/SubmitStationSequenceTask.srv"
  "srv/EstimateStationSequence.srv"
  "srv/ListDocks.srv"
  "srv/RequestDocking.srv"
  "srv/RequestUndocking.srv"
  "srv/GetDockState.srv"
  "srv/RequestCharging.srv"
  "srv/RequestOpportunityCharging.srv"
  "srv/BlockStationRoute.srv"
  "srv/DetectTrafficDeadlock.srv"
  "srv/UnblockStationRoute.srv"
  "srv/ListTrafficReservations.srv"
  "srv/ClearTrafficReservation.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
