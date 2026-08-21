#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/station_sequence_workflow.hpp"

namespace robot_tasks {

struct StationSequenceBehaviorTreeInput {
  std::vector<StationSequenceLeg> legs;
  std::unordered_set<std::string> active_or_queued_mission_ids;
  std::vector<MissionPreflightResult> preflights;
  int priority = 0;
  std::uint64_t first_sequence = 0;
};

struct StationSequenceBehaviorTreeResult {
  bool success = false;
  std::string branch;
  std::string message;
  std::vector<std::string> mission_ids;
  std::vector<QueuedMission> queued_missions;
};

std::string DefaultStationSequenceBehaviorTreeXml();
StationSequenceBehaviorTreeResult TickStationSequenceBehaviorTree(
    const StationSequenceBehaviorTreeInput& input);

}  // namespace robot_tasks
