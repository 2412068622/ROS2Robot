#include "robot_tasks/mission_localization_workflow.hpp"

#include <algorithm>

namespace robot_tasks {
namespace {

std::string PreviousStateOrUnknown(const std::string& state) {
  return state.empty() ? "UNKNOWN" : state;
}

std::string ReasonOrDefault(const std::string& reason, const std::string& fallback) {
  return reason.empty() ? fallback : reason;
}

bool RecoveringFromLocalizationLoss(const std::string& state) {
  return state == "LOST" || state == "RELOCALIZING";
}

}  // namespace

MissionLocalizationDecision PlanLocalizationPoseUpdate(
    const MissionLocalizationPoseUpdateRequest& request) {
  MissionLocalizationDecision decision;
  decision.localization = request.current;
  decision.localization.covariance_xy =
      std::max(request.covariance_x, request.covariance_y);

  if (decision.localization.covariance_xy > request.covariance_threshold) {
    const auto message =
        "localization covariance too high: " +
        std::to_string(decision.localization.covariance_xy);
    const bool first_lost_event = request.current.state != "LOST";
    decision.localization.localized = false;
    decision.localization.state = "LOST";
    decision.localization.station_id.clear();
    decision.localization.message = message;

    if (first_lost_event) {
      decision.event.emit = true;
      decision.event.state = "LOCALIZATION_LOST";
      decision.event.previous_state = PreviousStateOrUnknown(request.current.state);
      decision.event.message = message;
      decision.event.recoverable = false;
    }

    if (request.pause_on_lost && request.mission_active && !request.runner_paused) {
      decision.localization.paused_mission = true;
      decision.side_effects.request_pause = true;
      decision.side_effects.set_runner_state = true;
      decision.side_effects.runner_state = "LOCALIZATION_LOST";
      decision.side_effects.runner_message = message;
      decision.side_effects.runner_recoverable = false;
    }
    return decision;
  }

  const bool recovering = RecoveringFromLocalizationLoss(request.current.state);
  decision.localization.localized = true;
  decision.localization.station_id = request.current.station_id;
  decision.localization.state = recovering ? "RECOVERED" : "OK";
  decision.localization.message =
      recovering ? "localization covariance recovered" : "localization covariance ok";
  decision.localization.paused_mission = false;

  if (recovering) {
    decision.event.emit = true;
    decision.event.state = "LOCALIZATION_RECOVERED";
    decision.event.previous_state = PreviousStateOrUnknown(request.current.state);
    decision.event.message = decision.localization.message;
    decision.event.recoverable = true;
    if (request.current.paused_mission && request.mission_active && request.runner_paused) {
      decision.side_effects.request_resume = true;
      decision.side_effects.set_runner_state = true;
      decision.side_effects.runner_state = "RUNNING";
      decision.side_effects.runner_message = "localization recovered; mission resume requested";
      decision.side_effects.runner_recoverable = true;
    }
  }
  return decision;
}

MissionLocalizationDecision PlanRelocalizationRequest(
    const MissionRelocalizationRequest& request) {
  MissionLocalizationDecision decision;
  decision.localization = request.current;
  decision.localization.localized = false;
  decision.localization.state = "RELOCALIZING";
  decision.localization.station_id.clear();
  decision.localization.message =
      ReasonOrDefault(request.reason, "operator requested relocalization");

  decision.event.emit = true;
  decision.event.state = "RELOCALIZING";
  decision.event.previous_state = PreviousStateOrUnknown(request.current.state);
  decision.event.message = decision.localization.message;
  decision.event.recoverable = true;
  return decision;
}

MissionLocalizationDecision PlanInitialPoseRecovery(
    const MissionInitialPoseRecoveryRequest& request) {
  MissionLocalizationDecision decision;
  decision.localization = request.current;
  decision.localization.localized = true;
  decision.localization.state = "RECOVERED";
  decision.localization.station_id = request.station_id;
  decision.localization.covariance_xy = request.covariance_xy;
  decision.localization.message = "initial pose set from station " + request.station_id;
  decision.localization.paused_mission = false;

  decision.event.emit = true;
  decision.event.state = "LOCALIZATION_RECOVERED";
  decision.event.previous_state = PreviousStateOrUnknown(request.current.state);
  decision.event.message = decision.localization.message;
  decision.event.recoverable = true;

  if (request.current.paused_mission && request.mission_active && request.runner_paused) {
    decision.side_effects.request_resume = true;
    decision.side_effects.set_runner_state = true;
    decision.side_effects.runner_state = "RUNNING";
    decision.side_effects.runner_message = "localization recovered; mission resume requested";
    decision.side_effects.runner_recoverable = true;
  }
  return decision;
}

}  // namespace robot_tasks
