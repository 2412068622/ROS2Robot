#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "robot_tasks/fleet_catalog.hpp"
#include "robot_tasks/station_catalog.hpp"

namespace robot_tasks {

struct RuntimeFleetRobotState {
  bool update_enabled = false;
  bool enabled = true;
  bool update_state = false;
  std::string state;
  bool update_battery_voltage = false;
  double battery_voltage = 0.0;
  bool update_current_station_id = false;
  std::string current_station_id;
};

struct FleetRuntimeUpdateRequest {
  std::string robot_id;
  bool enabled = true;
  std::string state;
  double battery_voltage = 0.0;
  std::string current_station_id;
  bool update_enabled = false;
  bool update_state = false;
  bool update_battery_voltage = false;
  bool update_current_station_id = false;
};

struct FleetRuntimeUpdateDecision {
  bool success = false;
  bool mutate_state = false;
  RuntimeFleetRobotState runtime_state;
  FleetRobot effective_robot;
  std::string message;
};

struct FleetRobotListRequest {
  bool include_disabled = false;
  std::string capability;
};

struct FleetRobotListProjection {
  bool success = true;
  std::vector<std::string> robot_ids;
  std::vector<bool> enabled;
  std::vector<std::string> states;
  std::vector<double> battery_voltage;
  std::vector<std::string> current_station_ids;
  std::vector<std::string> capabilities;
  std::string message;
};

struct RemoteFleetTaskState {
  std::string task_id;
  std::string assigned_robot_id;
  std::string pickup_station;
  std::string dropoff_station;
  std::string state = "DISPATCHED_REMOTE";
  std::string message;
};

using RemoteFleetTaskMap = std::unordered_map<std::string, RemoteFleetTaskState>;

struct RemoteFleetTaskUpdateRequest {
  std::string task_id;
  std::string state;
  std::string message;
};

struct RemoteFleetTaskUpdateDecision {
  bool success = false;
  bool mutate_state = false;
  RemoteFleetTaskState task;
  std::string previous_state;
  std::string response_message;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state;
  std::string event_message;
  bool event_recoverable = true;
};

struct FleetStationTaskDispatchRequest {
  std::string task_id;
  std::string assigned_robot_id;
  std::string local_robot_id;
  std::string pickup_station;
  std::string dropoff_station;
  std::string previous_mission_state;
  double estimated_total_distance_m = 0.0;
};

struct FleetStationTaskDispatchDecision {
  bool success = false;
  bool dispatch_remote = false;
  bool mutate_remote_tasks = false;
  RemoteFleetTaskState remote_task;
  std::string station_order_id;
  std::string mission_id;
  std::string response_message;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state;
  std::string event_message;
  bool event_recoverable = true;
};

void ApplyRuntimeFleetState(
    FleetCatalog& catalog,
    const std::unordered_map<std::string, RuntimeFleetRobotState>& runtime_state);

FleetRuntimeUpdateDecision PlanFleetRuntimeStateUpdate(
    const FleetRuntimeUpdateRequest& request, const FleetCatalog& base_catalog,
    const StationCatalog* station_catalog,
    const std::unordered_map<std::string, RuntimeFleetRobotState>& runtime_state);

FleetRobotListProjection BuildFleetRobotListProjection(
    const FleetCatalog& catalog, const FleetRobotListRequest& request);

RemoteFleetTaskUpdateDecision PlanRemoteFleetTaskStateUpdate(
    const RemoteFleetTaskUpdateRequest& request, const RemoteFleetTaskMap& remote_tasks);

FleetStationTaskDispatchDecision PlanFleetStationTaskDispatch(
    const FleetStationTaskDispatchRequest& request, const RemoteFleetTaskMap& remote_tasks);

}  // namespace robot_tasks
