#include "robot_tasks/operator_snapshot.hpp"

#include <algorithm>

namespace robot_tasks {
namespace {

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

OperatorSnapshotProjection BuildOperatorSnapshotProjection(
    const OperatorSnapshotRequest& request) {
  OperatorSnapshotProjection projection;
  projection.mission_state = request.mission_state;
  projection.previous_mission_state = request.previous_mission_state;
  projection.active_mission_id = request.active_mission_id;
  projection.mission_message = request.mission_message;
  projection.recoverable = request.recoverable;
  projection.mission_active = request.mission_active;
  projection.paused = request.paused;
  projection.queue_size = static_cast<int>(request.mission_queue.size());

  for (const auto& mission : request.mission_queue) {
    projection.queued_mission_ids.push_back(mission.profile.mission_id);
    projection.queued_priorities.push_back(mission.priority);
  }
  projection.paused_workflow_order_ids.assign(
      request.paused_workflow_order_ids.begin(), request.paused_workflow_order_ids.end());
  std::sort(
      projection.paused_workflow_order_ids.begin(), projection.paused_workflow_order_ids.end());

  if (request.fleet != nullptr) {
    for (const auto& robot : request.fleet->robots) {
      projection.fleet_robot_ids.push_back(robot.id);
      projection.fleet_enabled.push_back(robot.enabled);
      projection.fleet_states.push_back(robot.state);
      projection.fleet_battery_voltage.push_back(robot.battery_voltage);
      projection.fleet_current_station_ids.push_back(robot.current_station_id);
      projection.fleet_capabilities.push_back(JoinCapabilities(robot.capabilities));
    }
  }

  if (request.remote_tasks != nullptr) {
    for (const auto& [task_id, task] : *request.remote_tasks) {
      projection.remote_task_ids.push_back(task_id);
      projection.remote_task_robot_ids.push_back(task.assigned_robot_id);
      projection.remote_task_states.push_back(task.state);
    }
  }

  if (request.facilities != nullptr) {
    const FacilityReservationMap empty_reservations;
    const auto& reservations =
        request.facility_reservations == nullptr ? empty_reservations : *request.facility_reservations;
    for (const auto& resource : request.facilities->resources) {
      projection.facility_resource_ids.push_back(resource.id);
      projection.facility_resource_types.push_back(resource.type);
      projection.facility_station_ids.push_back(resource.station_id);
      projection.facility_available.push_back(FacilityResourceAvailable(resource, reservations));
      projection.facility_status.push_back(FacilityResourceStatus(resource, reservations));
    }
  }

  if (request.business_orders != nullptr) {
    for (const auto& order_type : request.business_orders->order_types) {
      projection.business_types.push_back(order_type.business_type);
      projection.business_scenario_ids.push_back(order_type.scenario_id);
      projection.business_workflow_ids.push_back(order_type.workflow_id);
    }
  }

  const auto event_limit = NormalizeEventLimit(request.event_limit, 10U, 100U);
  const auto recent_events = SelectRecentMissionEvents(request.mission_events, event_limit);
  for (const auto& event : recent_events) {
    projection.recent_event_stamp.push_back(event.stamp);
    projection.recent_event_mission_ids.push_back(event.mission_id);
    projection.recent_event_states.push_back(event.state);
    projection.recent_event_messages.push_back(event.message);
  }

  projection.message = "operator snapshot queue_size=" +
                       std::to_string(projection.queue_size) + " robots=" +
                       std::to_string(projection.fleet_robot_ids.size()) + " resources=" +
                       std::to_string(projection.facility_resource_ids.size()) + " events=" +
                       std::to_string(projection.recent_event_states.size());
  return projection;
}

}  // namespace robot_tasks
