#pragma once

#include <string>

namespace robot_tasks {

struct MissionRecoveryConfig {
  bool enabled = true;
  std::string policy = "retry_then_dock";
  int retry_limit = 1;
};

struct MissionRecoveryDecision {
  std::string action = "none";
  std::string message;
  bool recoverable = true;
};

MissionRecoveryDecision DecideMissionRecovery(
    const MissionRecoveryConfig& config, int retry_count, bool is_dock_mission,
    const std::string& mission_id);

}  // namespace robot_tasks
