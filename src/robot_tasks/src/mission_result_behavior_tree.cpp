#include "robot_tasks/mission_result_behavior_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace robot_tasks {
namespace {

struct MissionResultBtContext {
  MissionResultWorkflowInput input;
  MissionResultBehaviorTreeResult result;
};

bool IsCanceled(const MissionResultWorkflowInput& input) {
  return input.code == MissionActionResultCode::kCanceled || input.cancellation_requested;
}

bool IsSucceeded(const MissionResultWorkflowInput& input) {
  return input.code == MissionActionResultCode::kSucceeded && input.result_success;
}

BT::NodeStatus PlanBranch(MissionResultBtContext& context, const std::string& branch) {
  context.result.decision = PlanMissionResultWorkflow(context.input);
  context.result.branch = context.result.decision.branch;
  return context.result.branch == branch ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
}

}  // namespace

std::string DefaultMissionResultBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="MissionResult">
  <BehaviorTree ID="MissionResult">
    <Fallback>
      <Sequence>
        <MissionWasCanceled/>
        <PlanCanceledMission/>
      </Sequence>
      <Sequence>
        <MissionSucceeded/>
        <PlanSucceededMission/>
      </Sequence>
      <PlanFailedMission/>
    </Fallback>
  </BehaviorTree>
</root>
)";
}

MissionResultBehaviorTreeResult TickMissionResultBehaviorTree(
    const MissionResultWorkflowInput& input) {
  MissionResultBtContext context;
  context.input = input;

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition(
      "MissionWasCanceled", [&context](BT::TreeNode&) {
        return IsCanceled(context.input) ? BT::NodeStatus::SUCCESS
                                         : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "PlanCanceledMission", [&context](BT::TreeNode&) {
        return PlanBranch(context, "canceled");
      });
  factory.registerSimpleCondition(
      "MissionSucceeded", [&context](BT::TreeNode&) {
        return IsSucceeded(context.input) ? BT::NodeStatus::SUCCESS
                                          : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "PlanSucceededMission", [&context](BT::TreeNode&) {
        return PlanBranch(context, "succeeded");
      });
  factory.registerSimpleAction(
      "PlanFailedMission", [&context](BT::TreeNode&) {
        return PlanBranch(context, "failed");
      });

  auto tree = factory.createTreeFromText(DefaultMissionResultBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace robot_tasks
