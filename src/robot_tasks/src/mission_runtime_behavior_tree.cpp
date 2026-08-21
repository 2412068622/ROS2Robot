#include "robot_tasks/mission_runtime_behavior_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace robot_tasks {
namespace {

struct MissionRuntimeBtContext {
  MissionRuntimeBehaviorTreeInput input;
  MissionRuntimeBehaviorTreeResult result;
};

}  // namespace

std::string DefaultMissionRuntimeBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="MissionRuntimeTick">
  <BehaviorTree ID="MissionRuntimeTick">
    <Sequence>
      <DecideMissionRuntimeTick/>
      <Fallback>
        <Sequence>
          <ShouldAutostartMission/>
          <ShouldStartQueuedMission/>
          <StageAutostartThenQueue/>
        </Sequence>
        <Sequence>
          <ShouldAutostartMission/>
          <StageAutostart/>
        </Sequence>
        <Sequence>
          <ShouldStartQueuedMission/>
          <StageQueueStart/>
        </Sequence>
        <StagePublishOnly/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

MissionRuntimeBehaviorTreeResult TickMissionRuntimeBehaviorTree(
    const MissionRuntimeBehaviorTreeInput& input) {
  MissionRuntimeBtContext context;
  context.input = input;

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction(
      "DecideMissionRuntimeTick", [&context](BT::TreeNode&) {
        context.result.decision = DecideMissionRuntimeTick(context.input.tick);
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "ShouldAutostartMission", [&context](BT::TreeNode&) {
        return context.result.decision.start_default_mission ? BT::NodeStatus::SUCCESS
                                                             : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleCondition(
      "ShouldStartQueuedMission", [&context](BT::TreeNode&) {
        return context.result.decision.start_queued_mission ? BT::NodeStatus::SUCCESS
                                                            : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageAutostartThenQueue", [&context](BT::TreeNode&) {
        context.result.branch = "autostart_then_queue_if_still_idle";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StageAutostart", [&context](BT::TreeNode&) {
        context.result.branch = "autostart";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StageQueueStart", [&context](BT::TreeNode&) {
        context.result.branch = "queue_start";
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StagePublishOnly", [&context](BT::TreeNode&) {
        context.result.branch = "publish_only";
        return BT::NodeStatus::SUCCESS;
      });

  auto tree = factory.createTreeFromText(DefaultMissionRuntimeBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace robot_tasks
