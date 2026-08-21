cmake_minimum_required(VERSION 3.20)
project(robot_interfaces_business)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SubmitOrder.srv"
  "srv/SubmitTransportOrder.srv"
  "srv/SubmitBusinessOrder.srv"
  "srv/ListBusinessOrderTypes.srv"
  "srv/GetOperatorSnapshot.srv"
  "srv/ListScenarioTasks.srv"
  "srv/SubmitScenarioTask.srv"
  "srv/ListScenarioWorkflows.srv"
  "srv/SubmitScenarioWorkflow.srv"
  "srv/SubmitVda5050Order.srv"
  "srv/SubmitPickingWave.srv"
  "srv/SubmitReplenishmentOrder.srv"
  "srv/SubmitMilkRun.srv"
  "srv/ConfirmPick.srv"
  "srv/ConfirmDropoff.srv"
  "srv/ReportShortPick.srv"
  "srv/GetOrderFulfillmentStatus.srv"
  "srv/CancelWorkflow.srv"
  "srv/PauseWorkflow.srv"
  "srv/ResumeWorkflow.srv"
  "srv/GetWorkflowStatus.srv"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
