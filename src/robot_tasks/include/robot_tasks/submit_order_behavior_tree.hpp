#pragma once

#include <optional>
#include <string>

#include "robot_tasks/submit_order_router.hpp"

namespace robot_tasks {

struct SubmitOrderBehaviorTreeInput {
  SubmitOrderInput request;
};

struct SubmitOrderBehaviorTreeResult {
  bool success = false;
  std::string branch;
  std::string message;
  std::optional<SubmitOrderRoute> route;
};

std::string DefaultSubmitOrderBehaviorTreeXml();
SubmitOrderBehaviorTreeResult TickSubmitOrderBehaviorTree(
    const SubmitOrderBehaviorTreeInput& input);

}  // namespace robot_tasks
