#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace robot_tasks {

struct StationConfirmationRuntimeState {
  std::string confirmation_id;
  std::string station_id;
  std::string action;
  std::string mission_id;
  std::string operator_hint;
  std::string state = "PENDING";
  int timeout_sec = 0;
  double created_at_sec = 0.0;
  std::string timeout_policy = "manual";
};

using StationConfirmationStateMap =
    std::unordered_map<std::string, StationConfirmationRuntimeState>;

struct StationConfirmationRequest {
  std::string confirmation_id;
  std::string station_id;
  std::string action;
  std::string mission_id;
  std::string operator_hint;
  int timeout_sec = 0;
  std::string timeout_policy = "manual";
  double created_at_sec = 0.0;
  bool station_known = false;
};

struct StationConfirmationRequestDecision {
  bool success = false;
  bool mutate_state = false;
  StationConfirmationRuntimeState confirmation;
  std::string confirmation_id;
  std::string state = "REJECTED";
  std::string message;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state = "READY";
  std::string event_message;
  bool event_recoverable = true;
};

struct StationConfirmationResolveRequest {
  std::string confirmation_id;
  std::string state;
  std::string operator_id;
  std::string note;
};

struct StationConfirmationResolveDecision {
  bool success = false;
  bool mutate_state = false;
  std::string confirmation_id;
  std::string state = "UNKNOWN";
  std::string message;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state = "PENDING";
  std::string event_message;
  bool event_recoverable = true;
};

struct StationConfirmationTimeoutDecision {
  bool timed_out = false;
  bool update_created_at = false;
  double new_created_at_sec = 0.0;
  bool update_state = false;
  std::string state;
  std::string event_mission_id;
  std::string event_state;
  std::string event_previous_state = "PENDING";
  std::string event_message;
  bool event_recoverable = true;
  bool request_dock_return = false;
  bool set_runner_state = false;
  std::string runner_state;
  std::string runner_message;
  bool runner_recoverable = true;
};

struct PendingStationConfirmationsProjection {
  bool success = true;
  std::vector<std::string> confirmation_ids;
  std::vector<std::string> station_ids;
  std::vector<std::string> actions;
  std::vector<std::string> mission_ids;
  std::vector<std::string> operator_hints;
  std::vector<std::string> states;
  std::string message;
};

std::string NormalizeStationConfirmationTimeoutPolicy(const std::string& policy);

StationConfirmationRequestDecision PlanStationConfirmationRequest(
    const StationConfirmationRequest& request, const StationConfirmationStateMap& confirmations);

StationConfirmationResolveDecision PlanStationConfirmationResolve(
    const StationConfirmationResolveRequest& request,
    const StationConfirmationStateMap& confirmations);

StationConfirmationTimeoutDecision PlanStationConfirmationTimeout(
    const StationConfirmationRuntimeState& confirmation, double current_time_sec);

PendingStationConfirmationsProjection BuildPendingStationConfirmationsProjection(
    const StationConfirmationStateMap& confirmations, const std::string& station_filter);

}  // namespace robot_tasks
