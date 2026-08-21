#pragma once

#include <string>

#include "robot_tasks/mission_result_workflow.hpp"

namespace robot_tasks {

struct MissionResultBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionResultWorkflowDecision decision;
};

std::string DefaultMissionResultBehaviorTreeXml();
MissionResultBehaviorTreeResult TickMissionResultBehaviorTree(
    const MissionResultWorkflowInput& input);

}  // namespace robot_tasks
