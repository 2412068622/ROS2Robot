#include "robot_tasks/fleet_runtime_state.hpp"

#include <cmath>

namespace robot_tasks {
namespace {

FleetRuntimeUpdateDecision Reject(const std::string& robot_id, const std::string& message) {
  FleetRuntimeUpdateDecision decision;
  decision.success = false;
  decision.mutate_state = false;
  decision.effective_robot.id = robot_id;
  decision.message = message;
  return decision;
}

RemoteFleetTaskUpdateDecision RejectRemoteTask(
    const std::string& task_id, const std::string& message) {
  RemoteFleetTaskUpdateDecision decision;
  decision.success = false;
  decision.mutate_state = false;
  decision.task.task_id = task_id;
  decision.response_message = message;
  return decision;
}

FleetStationTaskDispatchDecision RejectFleetStationTaskDispatch(const std::string& message) {
  FleetStationTaskDispatchDecision decision;
  decision.success = false;
  decision.response_message = message;
  return decision;
}

std::string JoinCapabilities(const std::vector<std::string>& capabilities) {
  std::string result;
  for (const auto& capability : capabilities) {
    if (!result.empty()) {
      result += ",";
    }
    result += capability;
  }
  return result;
}

}  // namespace

void ApplyRuntimeFleetState(
    FleetCatalog& catalog,
    const std::unordered_map<std::string, RuntimeFleetRobotState>& runtime_state) {
  for (auto& robot : catalog.robots) {
    const auto override_it = runtime_state.find(robot.id);
    if (override_it == runtime_state.end()) {
      continue;
    }
    const auto& state = override_it->second;
    if (state.update_enabled) {
      robot.enabled = state.enabled;
    }
    if (state.update_state) {
      robot.state = state.state;
    }
    if (state.update_battery_voltage) {
      robot.battery_voltage = state.battery_voltage;
    }
    if (state.update_current_station_id) {
      robot.current_station_id = state.current_station_id;
    }
  }
}

FleetRuntimeUpdateDecision PlanFleetRuntimeStateUpdate(
    const FleetRuntimeUpdateRequest& request, const FleetCatalog& base_catalog,
    const StationCatalog* station_catalog,
    const std::unordered_map<std::string, RuntimeFleetRobotState>& runtime_state) {
  if (request.robot_id.empty()) {
    return Reject("", "robot_id is empty");
  }

  const auto* base_robot = FindFleetRobot(base_catalog, request.robot_id);
  if (base_robot == nullptr) {
    return Reject(request.robot_id, "unknown fleet robot: " + request.robot_id);
  }

  RuntimeFleetRobotState next_state;
  const auto existing = runtime_state.find(request.robot_id);
  if (existing != runtime_state.end()) {
    next_state = existing->second;
  }

  if (request.update_enabled) {
    next_state.update_enabled = true;
    next_state.enabled = request.enabled;
  }
  if (request.update_state) {
    if (request.state.empty()) {
      return Reject(request.robot_id, "state is empty while update_state is true");
    }
    next_state.update_state = true;
    next_state.state = request.state;
  }
  if (request.update_battery_voltage) {
    if (!std::isfinite(request.battery_voltage) || request.battery_voltage < 0.0) {
      return Reject(request.robot_id, "battery_voltage must be finite and non-negative");
    }
    next_state.update_battery_voltage = true;
    next_state.battery_voltage = request.battery_voltage;
  }
  if (request.update_current_station_id) {
    if (request.current_station_id.empty()) {
      return Reject(
          request.robot_id, "current_station_id is empty while update_current_station_id is true");
    }
    if (station_catalog == nullptr ||
        FindStation(*station_catalog, request.current_station_id) == nullptr) {
      return Reject(request.robot_id, "unknown current_station_id: " + request.current_station_id);
    }
    next_state.update_current_station_id = true;
    next_state.current_station_id = request.current_station_id;
  }

  auto effective_catalog = base_catalog;
  auto next_runtime_state = runtime_state;
  next_runtime_state[request.robot_id] = next_state;
  ApplyRuntimeFleetState(effective_catalog, next_runtime_state);
  const auto* effective_robot = FindFleetRobot(effective_catalog, request.robot_id);

  FleetRuntimeUpdateDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.runtime_state = next_state;
  if (effective_robot != nullptr) {
    decision.effective_robot = *effective_robot;
  } else {
    decision.effective_robot = *base_robot;
  }
  decision.message = "updated runtime fleet state for " + request.robot_id;
  return decision;
}

FleetRobotListProjection BuildFleetRobotListProjection(
    const FleetCatalog& catalog, const FleetRobotListRequest& request) {
  FleetRobotListProjection projection;
  for (const auto& robot : catalog.robots) {
    if (!request.include_disabled && !robot.enabled) {
      continue;
    }
    if (!RobotHasCapability(robot, request.capability)) {
      continue;
    }
    projection.robot_ids.push_back(robot.id);
    projection.enabled.push_back(robot.enabled);
    projection.states.push_back(robot.state);
    projection.battery_voltage.push_back(robot.battery_voltage);
    projection.current_station_ids.push_back(robot.current_station_id);
    projection.capabilities.push_back(JoinCapabilities(robot.capabilities));
  }
  projection.message =
      "loaded " + std::to_string(projection.robot_ids.size()) + " fleet robot(s)";
  return projection;
}

RemoteFleetTaskUpdateDecision PlanRemoteFleetTaskStateUpdate(
    const RemoteFleetTaskUpdateRequest& request, const RemoteFleetTaskMap& remote_tasks) {
  if (request.task_id.empty() || request.state.empty()) {
    return RejectRemoteTask(request.task_id, "task_id or state is empty");
  }
  const auto task = remote_tasks.find(request.task_id);
  if (task == remote_tasks.end()) {
    return RejectRemoteTask(request.task_id, "unknown remote task: " + request.task_id);
  }

  RemoteFleetTaskUpdateDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.task = task->second;
  decision.previous_state = task->second.state;
  decision.task.state = request.state;
  decision.task.message =
      request.message.empty() ? "remote task state updated" : request.message;
  decision.response_message = "updated remote task " + request.task_id + " to " + request.state;
  decision.event_mission_id = "remote_fleet_task_" + request.task_id;
  decision.event_state = request.state;
  decision.event_previous_state = decision.previous_state;
  decision.event_message = decision.task.message;
  decision.event_recoverable = true;
  return decision;
}

FleetStationTaskDispatchDecision PlanFleetStationTaskDispatch(
    const FleetStationTaskDispatchRequest& request, const RemoteFleetTaskMap& remote_tasks) {
  if (request.task_id.empty()) {
    return RejectFleetStationTaskDispatch("fleet task_id is empty");
  }
  if (request.assigned_robot_id.empty()) {
    return RejectFleetStationTaskDispatch("assigned robot id is empty");
  }

  if (request.assigned_robot_id == request.local_robot_id) {
    FleetStationTaskDispatchDecision decision;
    decision.success = true;
    decision.dispatch_remote = false;
    decision.mutate_remote_tasks = false;
    decision.station_order_id = "fleet_" + request.task_id;
    return decision;
  }

  if (remote_tasks.find(request.task_id) != remote_tasks.end()) {
    return RejectFleetStationTaskDispatch("remote fleet task already exists: " + request.task_id);
  }

  FleetStationTaskDispatchDecision decision;
  decision.success = true;
  decision.dispatch_remote = true;
  decision.mutate_remote_tasks = true;
  decision.remote_task = RemoteFleetTaskState{
      request.task_id,
      request.assigned_robot_id,
      request.pickup_station,
      request.dropoff_station,
      "DISPATCHED_REMOTE",
      "dispatched remote fleet task " + request.task_id + " to " +
          request.assigned_robot_id};
  decision.mission_id = "remote_fleet_task_" + request.task_id;
  decision.response_message =
      decision.remote_task.message + " estimated_total_distance_m=" +
      std::to_string(request.estimated_total_distance_m);
  decision.event_mission_id = decision.mission_id;
  decision.event_state = decision.remote_task.state;
  decision.event_previous_state = request.previous_mission_state;
  decision.event_message = decision.remote_task.message;
  decision.event_recoverable = true;
  return decision;
}

}  // namespace robot_tasks
