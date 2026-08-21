#include "robot_tasks/mission_queue_behavior_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace robot_tasks {
namespace {

struct MissionQueueBtContext {
  MissionQueueBehaviorTreeInput input;
  MissionQueueBehaviorTreeResult result;
};

MissionQueueBtContext MakeMissionQueueContext(const MissionQueueBehaviorTreeInput& input) {
  MissionQueueBtContext context;
  context.input = input;
  context.result.dispatch.remaining_queue = input.queue;
  return context;
}

}  // namespace

std::string DefaultMissionQueueBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="MissionQueueDispatch">
  <BehaviorTree ID="MissionQueueDispatch">
    <Sequence>
      <CheckQueueNotEmpty/>
      <CheckRunnableMission/>
      <PopNextRunnableMission/>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

MissionQueueBehaviorTreeResult TickMissionQueueBehaviorTree(
    const MissionQueueBehaviorTreeInput& input) {
  auto context = MakeMissionQueueContext(input);

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition(
      "CheckQueueNotEmpty", [&context](BT::TreeNode&) {
        if (!context.input.queue.empty()) {
          return BT::NodeStatus::SUCCESS;
        }
        context.result.dispatch.queue_empty = true;
        context.result.dispatch.message = "mission queue is empty";
        return BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleCondition(
      "CheckRunnableMission", [&context](BT::TreeNode&) {
        const auto next_index =
            FindNextRunnableMissionIndex(context.input.queue, context.input.paused_order_ids);
        if (next_index.has_value()) {
          return BT::NodeStatus::SUCCESS;
        }
        context.result.dispatch.all_paused = true;
        context.result.dispatch.message = "all queued workflow missions are paused";
        return BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "PopNextRunnableMission", [&context](BT::TreeNode&) {
        context.result.dispatch = PopNextRunnableMission(
            context.input.queue, context.input.paused_order_ids);
        context.result.branch = "mission_queue_dispatch";
        return context.result.dispatch.success ? BT::NodeStatus::SUCCESS
                                               : BT::NodeStatus::FAILURE;
      });

  auto tree = factory.createTreeFromText(DefaultMissionQueueBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace robot_tasks
