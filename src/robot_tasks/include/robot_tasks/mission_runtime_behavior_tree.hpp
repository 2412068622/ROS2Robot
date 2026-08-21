#pragma once

#include <string>

#include "robot_tasks/mission_runtime_workflow.hpp"

namespace robot_tasks {

struct MissionRuntimeBehaviorTreeInput {
  MissionRuntimeTickInput tick;
};

struct MissionRuntimeBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionRuntimeTickDecision decision;
};

std::string DefaultMissionRuntimeBehaviorTreeXml();
MissionRuntimeBehaviorTreeResult TickMissionRuntimeBehaviorTree(
    const MissionRuntimeBehaviorTreeInput& input);

}  // namespace robot_tasks
