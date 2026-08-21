#pragma once

#include <string>

#include "robot_tasks/mission_queue.hpp"

namespace robot_tasks {

struct QueuedMissionStartBehaviorTreeInput {
  QueuedMissionStartRequest request;
};

struct QueuedMissionStartBehaviorTreeResult {
  bool success = false;
  std::string branch;
  QueuedMissionStartDecision decision;
};

std::string DefaultQueuedMissionStartBehaviorTreeXml();
QueuedMissionStartBehaviorTreeResult TickQueuedMissionStartBehaviorTree(
    const QueuedMissionStartBehaviorTreeInput& input);

}  // namespace robot_tasks
