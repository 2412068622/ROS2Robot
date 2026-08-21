#include "robot_tasks/mission_recovery_behavior_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace robot_tasks {
namespace {

struct MissionRecoveryBtContext {
  MissionRecoveryBehaviorTreeInput input;
  MissionRecoveryBehaviorTreeResult result;
};

MissionRecoveryBtContext MakeMissionRecoveryContext(
    const MissionRecoveryBehaviorTreeInput& input) {
  MissionRecoveryBtContext context;
  context.input = input;
  context.result.retry_count = input.retry_count;
  context.result.next_queue_sequence = input.next_queue_sequence;
  return context;
}

std::string BuildStateMessage(
    const MissionRecoveryDecision& decision, const std::string& failure_message) {
  return decision.message + ": " + failure_message;
}

bool HasAction(const MissionRecoveryBtContext& context, const std::string& action) {
  return context.result.decision.action == action;
}

void SetRecoveryState(
    MissionRecoveryBtContext& context, const std::string& branch,
    const std::string& state) {
  context.result.branch = branch;
  context.result.state = state;
  context.result.state_message =
      BuildStateMessage(context.result.decision, context.input.failure_message);
  context.result.recoverable = context.result.decision.recoverable;
}

}  // namespace

std::string DefaultMissionRecoveryBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="MissionRecovery">
  <BehaviorTree ID="MissionRecovery">
    <Sequence>
      <DecideMissionRecovery/>
      <Fallback>
        <Sequence>
          <ShouldRetryMission/>
          <StageRetryMission/>
        </Sequence>
        <Sequence>
          <ShouldReturnToDock/>
          <StageDockRecovery/>
        </Sequence>
        <Sequence>
          <ShouldRequestManualIntervention/>
          <StageManualIntervention/>
        </Sequence>
        <StageFailedMission/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

MissionRecoveryBehaviorTreeResult TickMissionRecoveryBehaviorTree(
    const MissionRecoveryBehaviorTreeInput& input) {
  auto context = MakeMissionRecoveryContext(input);

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction(
      "DecideMissionRecovery", [&context](BT::TreeNode&) {
        context.result.decision = DecideMissionRecovery(
            context.input.config, context.input.retry_count, context.input.is_dock_mission,
            context.input.mission_id);
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "ShouldRetryMission", [&context](BT::TreeNode&) {
        return HasAction(context, "retry") ? BT::NodeStatus::SUCCESS
                                           : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageRetryMission", [&context](BT::TreeNode&) {
        if (!context.input.active_profile.has_value()) {
          return BT::NodeStatus::FAILURE;
        }
        SetRecoveryState(context, "retry", "RECOVERING");
        QueuedMission mission;
        mission.profile = *context.input.active_profile;
        mission.mission_file = context.input.mission_file;
        mission.priority = context.input.recovery_priority;
        mission.sequence = context.result.next_queue_sequence++;
        context.result.retry_mission = mission;
        ++context.result.retry_count;
        context.result.queue_running = true;
        context.result.queue_start_requested = true;
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "ShouldReturnToDock", [&context](BT::TreeNode&) {
        return HasAction(context, "dock") ? BT::NodeStatus::SUCCESS
                                          : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageDockRecovery", [&context](BT::TreeNode&) {
        SetRecoveryState(context, "dock", "RECOVERING");
        context.result.request_dock_return = true;
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "ShouldRequestManualIntervention", [&context](BT::TreeNode&) {
        return HasAction(context, "manual") ? BT::NodeStatus::SUCCESS
                                            : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageManualIntervention", [&context](BT::TreeNode&) {
        SetRecoveryState(context, "manual", "NEEDS_OPERATOR");
        context.result.stop_queue = true;
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "StageFailedMission", [&context](BT::TreeNode&) {
        SetRecoveryState(context, "failed", "FAILED");
        context.result.stop_queue = true;
        return BT::NodeStatus::SUCCESS;
      });

  auto tree = factory.createTreeFromText(DefaultMissionRecoveryBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace robot_tasks
