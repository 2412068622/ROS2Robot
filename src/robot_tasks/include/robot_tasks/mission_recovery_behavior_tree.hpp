#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "robot_tasks/mission_queue.hpp"
#include "robot_tasks/mission_recovery_policy.hpp"

namespace robot_tasks {

struct MissionRecoveryBehaviorTreeInput {
  MissionRecoveryConfig config;
  int retry_count = 0;
  bool is_dock_mission = false;
  std::string mission_id;
  std::string failure_message;
  std::optional<MissionProfile> active_profile;
  std::string mission_file;
  int recovery_priority = 0;
  std::uint64_t next_queue_sequence = 0;
};

struct MissionRecoveryBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionRecoveryDecision decision;
  std::string state;
  std::string state_message;
  bool recoverable = false;
  int retry_count = 0;
  std::uint64_t next_queue_sequence = 0;
  bool queue_running = false;
  bool queue_start_requested = false;
  bool request_dock_return = false;
  bool stop_queue = false;
  std::optional<QueuedMission> retry_mission;
};

std::string DefaultMissionRecoveryBehaviorTreeXml();
MissionRecoveryBehaviorTreeResult TickMissionRecoveryBehaviorTree(
    const MissionRecoveryBehaviorTreeInput& input);

}  // namespace robot_tasks
