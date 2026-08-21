#include "robot_tasks/station_confirmation_workflow.hpp"

namespace robot_tasks {
namespace {

std::string TimeoutMessage(const StationConfirmationRuntimeState& confirmation) {
  return "station confirmation timed out at " + confirmation.station_id + " action " +
         confirmation.action + " policy " + confirmation.timeout_policy;
}

StationConfirmationTimeoutDecision BaseTimeoutDecision(
    const StationConfirmationRuntimeState& confirmation) {
  StationConfirmationTimeoutDecision decision;
  decision.timed_out = true;
  decision.event_mission_id = "station_confirmation_" + confirmation.confirmation_id;
  decision.event_message = TimeoutMessage(confirmation);
  return decision;
}

StationConfirmationRequestDecision RejectRequest(
    const std::string& confirmation_id, const std::string& message) {
  StationConfirmationRequestDecision decision;
  decision.success = false;
  decision.mutate_state = false;
  decision.confirmation_id = confirmation_id;
  decision.state = "REJECTED";
  decision.message = message;
  return decision;
}

StationConfirmationResolveDecision RejectResolve(
    const std::string& confirmation_id, const std::string& state,
    const std::string& message) {
  StationConfirmationResolveDecision decision;
  decision.success = false;
  decision.mutate_state = false;
  decision.confirmation_id = confirmation_id;
  decision.state = state;
  decision.message = message;
  return decision;
}

}  // namespace

std::string NormalizeStationConfirmationTimeoutPolicy(const std::string& policy) {
  if (policy == "retry" || policy == "skip" || policy == "return_to_dock" ||
      policy == "manual") {
    return policy;
  }
  return "manual";
}

StationConfirmationRequestDecision PlanStationConfirmationRequest(
    const StationConfirmationRequest& request,
    const StationConfirmationStateMap& confirmations) {
  if (request.station_id.empty()) {
    return RejectRequest(request.confirmation_id, "station_id is empty");
  }
  if (!request.station_known) {
    return RejectRequest(request.confirmation_id, "unknown station: " + request.station_id);
  }
  const auto existing = confirmations.find(request.confirmation_id);
  if (existing != confirmations.end() && existing->second.state == "PENDING") {
    StationConfirmationRequestDecision decision =
        RejectRequest(request.confirmation_id,
                      "confirmation already pending: " + request.confirmation_id);
    decision.state = existing->second.state;
    return decision;
  }

  StationConfirmationRequestDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.confirmation_id = request.confirmation_id;
  decision.state = "PENDING";
  decision.message = "station confirmation pending: " + request.confirmation_id;
  decision.confirmation.confirmation_id = request.confirmation_id;
  decision.confirmation.station_id = request.station_id;
  decision.confirmation.action = request.action;
  decision.confirmation.mission_id = request.mission_id;
  decision.confirmation.operator_hint = request.operator_hint;
  decision.confirmation.state = "PENDING";
  decision.confirmation.timeout_sec = request.timeout_sec < 0 ? 0 : request.timeout_sec;
  decision.confirmation.created_at_sec = request.created_at_sec;
  decision.confirmation.timeout_policy =
      NormalizeStationConfirmationTimeoutPolicy(request.timeout_policy);
  decision.event_mission_id = "station_confirmation_" + request.confirmation_id;
  decision.event_state = "CONFIRMATION_PENDING";
  decision.event_message = "pending " + request.action + " at " + request.station_id;
  return decision;
}

StationConfirmationResolveDecision PlanStationConfirmationResolve(
    const StationConfirmationResolveRequest& request,
    const StationConfirmationStateMap& confirmations) {
  const auto confirmation = confirmations.find(request.confirmation_id);
  if (confirmation == confirmations.end()) {
    return RejectResolve(
        request.confirmation_id, "UNKNOWN", "unknown confirmation: " + request.confirmation_id);
  }
  if (confirmation->second.state != "PENDING") {
    return RejectResolve(
        request.confirmation_id, confirmation->second.state,
        "confirmation already resolved: " + request.confirmation_id);
  }

  StationConfirmationResolveDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.confirmation_id = request.confirmation_id;
  decision.state = request.state;
  decision.message = request.note;
  decision.event_mission_id = "station_confirmation_" + request.confirmation_id;
  decision.event_state = "CONFIRMATION_" + request.state;
  decision.event_message = request.note + " by " + request.operator_id;
  decision.event_recoverable = request.state == "CONFIRMED";
  return decision;
}

StationConfirmationTimeoutDecision PlanStationConfirmationTimeout(
    const StationConfirmationRuntimeState& confirmation, const double current_time_sec) {
  StationConfirmationTimeoutDecision decision;
  if (confirmation.state != "PENDING" || confirmation.timeout_sec <= 0 ||
      current_time_sec - confirmation.created_at_sec < confirmation.timeout_sec) {
    return decision;
  }

  const auto policy = NormalizeStationConfirmationTimeoutPolicy(confirmation.timeout_policy);
  decision = BaseTimeoutDecision(confirmation);
  if (policy == "retry") {
    decision.update_created_at = true;
    decision.new_created_at_sec = current_time_sec;
    decision.event_state = "CONFIRMATION_TIMEOUT_RETRY";
    decision.event_recoverable = true;
    return decision;
  }
  if (policy == "skip") {
    decision.update_state = true;
    decision.state = "SKIPPED";
    decision.event_state = "CONFIRMATION_SKIPPED";
    decision.event_recoverable = true;
    return decision;
  }
  if (policy == "return_to_dock") {
    decision.update_state = true;
    decision.state = "TIMED_OUT";
    decision.event_state = "CONFIRMATION_TIMED_OUT";
    decision.event_recoverable = true;
    decision.request_dock_return = true;
    return decision;
  }

  decision.update_state = true;
  decision.state = "TIMED_OUT";
  decision.event_state = "CONFIRMATION_TIMED_OUT";
  decision.event_recoverable = false;
  decision.set_runner_state = true;
  decision.runner_state = "NEEDS_OPERATOR";
  decision.runner_message = decision.event_message;
  decision.runner_recoverable = true;
  return decision;
}

PendingStationConfirmationsProjection BuildPendingStationConfirmationsProjection(
    const StationConfirmationStateMap& confirmations, const std::string& station_filter) {
  PendingStationConfirmationsProjection projection;
  for (const auto& [id, confirmation] : confirmations) {
    if (confirmation.state != "PENDING") {
      continue;
    }
    if (!station_filter.empty() && confirmation.station_id != station_filter) {
      continue;
    }
    projection.confirmation_ids.push_back(id);
    projection.station_ids.push_back(confirmation.station_id);
    projection.actions.push_back(confirmation.action);
    projection.mission_ids.push_back(confirmation.mission_id);
    projection.operator_hints.push_back(confirmation.operator_hint);
    projection.states.push_back(confirmation.state);
  }
  projection.message =
      "loaded " + std::to_string(projection.confirmation_ids.size()) +
      " pending confirmation(s)";
  return projection;
}

}  // namespace robot_tasks
