#pragma once

#include <string>

namespace robot_tasks {

struct MissionLocalizationState {
  bool localized = true;
  std::string state = "OK";
  std::string station_id;
  std::string message = "localization covariance ok";
  double covariance_xy = 0.0;
  bool paused_mission = false;
};

struct MissionLocalizationEvent {
  bool emit = false;
  std::string mission_id = "localization";
  std::string state;
  std::string previous_state;
  std::string message;
  bool recoverable = false;
};

struct MissionLocalizationSideEffects {
  bool request_pause = false;
  bool request_resume = false;
  bool set_runner_state = false;
  std::string runner_state;
  std::string runner_message;
  bool runner_recoverable = true;
};

struct MissionLocalizationDecision {
  MissionLocalizationState localization;
  MissionLocalizationEvent event;
  MissionLocalizationSideEffects side_effects;
};

struct MissionLocalizationPoseUpdateRequest {
  MissionLocalizationState current;
  double covariance_x = 0.0;
  double covariance_y = 0.0;
  double covariance_threshold = 0.5;
  bool pause_on_lost = true;
  bool mission_active = false;
  bool runner_paused = false;
};

struct MissionRelocalizationRequest {
  MissionLocalizationState current;
  std::string reason;
};

struct MissionInitialPoseRecoveryRequest {
  MissionLocalizationState current;
  std::string station_id;
  bool mission_active = false;
  bool runner_paused = false;
  double covariance_xy = 0.05;
};

MissionLocalizationDecision PlanLocalizationPoseUpdate(
    const MissionLocalizationPoseUpdateRequest& request);
MissionLocalizationDecision PlanRelocalizationRequest(
    const MissionRelocalizationRequest& request);
MissionLocalizationDecision PlanInitialPoseRecovery(
    const MissionInitialPoseRecoveryRequest& request);

}  // namespace robot_tasks
