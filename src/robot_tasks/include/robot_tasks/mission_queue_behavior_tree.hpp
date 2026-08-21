#pragma once

#include <string>
#include <unordered_set>
#include <vector>

#include "robot_tasks/mission_queue.hpp"

namespace robot_tasks {

struct MissionQueueBehaviorTreeInput {
  std::vector<QueuedMission> queue;
  std::unordered_set<std::string> paused_order_ids;
};

struct MissionQueueBehaviorTreeResult {
  bool success = false;
  std::string branch;
  MissionQueueDispatchResult dispatch;
};

std::string DefaultMissionQueueBehaviorTreeXml();
MissionQueueBehaviorTreeResult TickMissionQueueBehaviorTree(
    const MissionQueueBehaviorTreeInput& input);

}  // namespace robot_tasks
