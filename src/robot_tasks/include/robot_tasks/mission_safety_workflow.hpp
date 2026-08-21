#pragma once

#include <string>

namespace robot_tasks {

struct MissionSafetyState {
  bool obstacle_blocked = false;
  bool blockage_paused_mission = false;
  double runtime_speed_limit_mps = 0.0;
  double zone_speed_limit_mps = 0.6;
  std::string stop_reason;
  std::string blockage_id;
  std::string mission_id;
  std::string message = "safety ok";
};

struct MissionSafetyEvent {
  bool emit = false;
  std::string mission_id;
  std::string state;
  std::string previous_state;
  std::string message;
  bool recoverable = true;
};

struct MissionSafetySideEffects {
  bool request_pause = false;
  bool request_resume = false;
  bool set_runner_state = false;
  std::string runner_state;
  std::string runner_message;
  bool runner_recoverable = true;
};

struct MissionSafetyDecision {
  bool success = false;
  bool mutate_state = false;
  MissionSafetyState safety;
  MissionSafetyEvent event;
  MissionSafetySideEffects side_effects;
  std::string response_message;
  std::string response_state;
  std::string response_blockage_id;
  double response_runtime_speed_limit_mps = 0.0;
  double response_effective_speed_limit_mps = 0.0;
};

struct MissionSafetySpeedLimitRequest {
  MissionSafetyState current;
  double speed_limit_mps = 0.0;
  std::string reason;
};

struct MissionSafetyReportBlockageRequest {
  MissionSafetyState current;
  std::string blockage_id;
  std::string reason;
  std::string active_mission_id;
  std::string runner_previous_state = "UNKNOWN";
  bool pause_active = false;
  bool pause_on_blockage = true;
  bool mission_active = false;
  bool runner_paused = false;
};

struct MissionSafetyClearBlockageRequest {
  MissionSafetyState current;
  std::string blockage_id;
  std::string resolution;
  std::string runner_state;
  bool resume_active = false;
  bool mission_active = false;
  bool runner_paused = false;
};

double EffectiveSafetySpeedLimit(const MissionSafetyState& state);
std::string MissionSafetyStateName(const MissionSafetyState& state);

MissionSafetyDecision PlanSetDynamicSpeedLimit(
    const MissionSafetySpeedLimitRequest& request);
MissionSafetyDecision PlanReportObstacleBlockage(
    const MissionSafetyReportBlockageRequest& request);
MissionSafetyDecision PlanClearObstacleBlockage(
    const MissionSafetyClearBlockageRequest& request);

}  // namespace robot_tasks
