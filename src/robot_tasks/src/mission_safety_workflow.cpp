#include "robot_tasks/mission_safety_workflow.hpp"

#include <algorithm>
#include <string>

namespace robot_tasks {
namespace {

std::string ReasonOrDefault(const std::string& reason, const std::string& fallback) {
  return reason.empty() ? fallback : reason;
}

MissionSafetyDecision RejectWithoutMutation(
    const MissionSafetyState& current, const std::string& message) {
  MissionSafetyDecision decision;
  decision.success = false;
  decision.mutate_state = false;
  decision.safety = current;
  decision.response_message = message;
  decision.response_state = MissionSafetyStateName(current);
  decision.response_blockage_id = current.blockage_id;
  decision.response_runtime_speed_limit_mps = current.runtime_speed_limit_mps;
  decision.response_effective_speed_limit_mps = EffectiveSafetySpeedLimit(current);
  return decision;
}

void FillCommonResponse(MissionSafetyDecision& decision) {
  decision.response_state = MissionSafetyStateName(decision.safety);
  decision.response_blockage_id = decision.safety.blockage_id;
  decision.response_runtime_speed_limit_mps = decision.safety.runtime_speed_limit_mps;
  decision.response_effective_speed_limit_mps = EffectiveSafetySpeedLimit(decision.safety);
}

}  // namespace

double EffectiveSafetySpeedLimit(const MissionSafetyState& state) {
  if (state.runtime_speed_limit_mps > 0.0) {
    return std::min(state.zone_speed_limit_mps, state.runtime_speed_limit_mps);
  }
  return state.zone_speed_limit_mps;
}

std::string MissionSafetyStateName(const MissionSafetyState& state) {
  if (state.obstacle_blocked) {
    return "OBSTACLE_BLOCKED";
  }
  if (state.runtime_speed_limit_mps > 0.0) {
    return "SPEED_LIMITED";
  }
  return "OK";
}

MissionSafetyDecision PlanSetDynamicSpeedLimit(
    const MissionSafetySpeedLimitRequest& request) {
  if (request.speed_limit_mps < 0.0) {
    return RejectWithoutMutation(request.current, "speed_limit_mps must be >= 0");
  }

  MissionSafetyDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.safety = request.current;
  decision.safety.runtime_speed_limit_mps = request.speed_limit_mps;
  decision.safety.message =
      ReasonOrDefault(request.reason, "dynamic speed limit updated");
  decision.safety.stop_reason =
      decision.safety.runtime_speed_limit_mps > 0.0 ? decision.safety.message
                                                    : request.current.stop_reason;
  if (!decision.safety.obstacle_blocked && decision.safety.runtime_speed_limit_mps <= 0.0) {
    decision.safety.stop_reason.clear();
  }

  decision.event.emit = true;
  decision.event.mission_id = "safety";
  decision.event.state = "DYNAMIC_SPEED_LIMIT";
  decision.event.previous_state = MissionSafetyStateName(decision.safety);
  decision.event.message = decision.safety.message;
  decision.event.recoverable = true;

  decision.response_message =
      decision.safety.runtime_speed_limit_mps > 0.0 ? decision.safety.message
                                                    : "dynamic speed limit cleared";
  FillCommonResponse(decision);
  return decision;
}

MissionSafetyDecision PlanReportObstacleBlockage(
    const MissionSafetyReportBlockageRequest& request) {
  if (request.current.obstacle_blocked) {
    return RejectWithoutMutation(
        request.current, "obstacle blockage already active: " + request.current.blockage_id);
  }

  MissionSafetyDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.safety = request.current;
  decision.safety.obstacle_blocked = true;
  decision.safety.blockage_id = request.blockage_id;
  decision.safety.blockage_paused_mission = false;
  decision.safety.stop_reason =
      ReasonOrDefault(request.reason, "obstacle blockage reported");
  decision.safety.mission_id = request.mission_active ? request.active_mission_id : "";
  decision.safety.message = decision.safety.stop_reason;

  if (request.pause_active && request.pause_on_blockage && request.mission_active &&
      !request.runner_paused) {
    decision.safety.blockage_paused_mission = true;
    decision.side_effects.request_pause = true;
    decision.side_effects.set_runner_state = true;
    decision.side_effects.runner_state = "SAFETY_BLOCKED";
    decision.side_effects.runner_message = decision.safety.message;
    decision.side_effects.runner_recoverable = false;
  }

  decision.event.emit = true;
  decision.event.mission_id = "safety_" + decision.safety.blockage_id;
  decision.event.state = "OBSTACLE_BLOCKED";
  decision.event.previous_state =
      request.runner_previous_state.empty() ? "UNKNOWN" : request.runner_previous_state;
  decision.event.message = decision.safety.message;
  decision.event.recoverable = false;

  decision.response_message = decision.safety.message;
  FillCommonResponse(decision);
  return decision;
}

MissionSafetyDecision PlanClearObstacleBlockage(
    const MissionSafetyClearBlockageRequest& request) {
  if (!request.current.obstacle_blocked) {
    return RejectWithoutMutation(request.current, "no obstacle blockage is active");
  }
  if (!request.blockage_id.empty() && request.blockage_id != request.current.blockage_id) {
    return RejectWithoutMutation(
        request.current, "active blockage mismatch: " + request.current.blockage_id);
  }

  const auto cleared_id = request.current.blockage_id;
  MissionSafetyDecision decision;
  decision.success = true;
  decision.mutate_state = true;
  decision.safety = request.current;
  decision.safety.obstacle_blocked = false;
  decision.safety.blockage_paused_mission = false;
  decision.safety.blockage_id.clear();
  decision.safety.stop_reason.clear();
  decision.safety.message =
      ReasonOrDefault(request.resolution, "obstacle blockage cleared");

  decision.event.emit = true;
  decision.event.mission_id = "safety_" + cleared_id;
  decision.event.state = "OBSTACLE_CLEARED";
  decision.event.previous_state = "OBSTACLE_BLOCKED";
  decision.event.message = decision.safety.message;
  decision.event.recoverable = true;

  if (request.resume_active && request.current.blockage_paused_mission &&
      request.mission_active && request.runner_paused) {
    decision.side_effects.request_resume = true;
    decision.side_effects.set_runner_state = true;
    decision.side_effects.runner_state = "RUNNING";
    decision.side_effects.runner_message = "safety blockage cleared; mission resume requested";
    decision.side_effects.runner_recoverable = true;
  } else if (!request.mission_active && request.runner_state == "SAFETY_BLOCKED") {
    decision.side_effects.set_runner_state = true;
    decision.side_effects.runner_state = "READY";
    decision.side_effects.runner_message = decision.safety.message;
    decision.side_effects.runner_recoverable = true;
  }

  decision.response_message = decision.safety.message;
  FillCommonResponse(decision);
  return decision;
}

}  // namespace robot_tasks
