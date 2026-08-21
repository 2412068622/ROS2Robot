#include "robot_tasks/docking_behavior_tree.hpp"

#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace robot_tasks {
namespace {

struct DockingBtContext {
  DockingBehaviorTreeInput input;
  DockingStateMap dock_states;
  DockingMissionMap dock_by_mission;
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
  DockingBehaviorTreeResult result;
};

DockingBtContext MakeContext(const DockingBehaviorTreeInput& input) {
  DockingBtContext context;
  context.input = input;
  context.dock_states[input.dock_state.dock_id] = input.dock_state;
  context.dock_by_mission[input.dock_state.mission_id] = input.dock_state.dock_id;
  context.reservations[input.resource_id] =
      ResourceReservation{input.dock_state.mission_id, input.dock_state.mission_id, "APPROACHING", true};
  context.resource_by_mission[input.dock_state.mission_id] = input.resource_id;
  context.result.dock_state = input.dock_state;
  context.result.reservation = context.reservations[input.resource_id];
  return context;
}

void CaptureResult(DockingBtContext& context, const std::string& branch) {
  const auto dock_it = context.dock_states.find(context.input.dock_state.dock_id);
  if (dock_it != context.dock_states.end()) {
    context.result.dock_state = dock_it->second;
  }
  const auto reservation_it = context.reservations.find(context.input.resource_id);
  if (reservation_it != context.reservations.end()) {
    context.result.reservation = reservation_it->second;
  }
  context.result.branch = branch;
}

}  // namespace

std::string DefaultDockingBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="DockingSubflow">
  <BehaviorTree ID="DockingSubflow">
    <Fallback>
      <Sequence>
        <NeedsContactRetry/>
        <StageContactRetry/>
      </Sequence>
      <CompleteDocking/>
    </Fallback>
  </BehaviorTree>
</root>
)";
}

DockingBehaviorTreeResult TickDockingBehaviorTree(const DockingBehaviorTreeInput& input) {
  auto context = MakeContext(input);

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition(
      "NeedsContactRetry", [&context](BT::TreeNode&) {
        const auto state_it = context.dock_states.find(context.input.dock_state.dock_id);
        if (state_it == context.dock_states.end()) {
          return BT::NodeStatus::FAILURE;
        }
        const auto& state = state_it->second;
        return (!state.simulate_contact_success && state.contact_attempts == 0)
                   ? BT::NodeStatus::SUCCESS
                   : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "StageContactRetry", [&context](BT::TreeNode&) {
        context.result.transition = StageDockingContactRetry(
            context.dock_states, context.dock_by_mission, context.reservations,
            context.resource_by_mission, context.input.dock_state.mission_id);
        CaptureResult(context, "contact_retry");
        return context.result.transition.changed ? BT::NodeStatus::SUCCESS
                                                 : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "CompleteDocking", [&context](BT::TreeNode&) {
        context.result.transition = CompleteDockingSubflow(
            context.dock_states, context.dock_by_mission, context.reservations,
            context.resource_by_mission, context.input.dock_state.mission_id);
        CaptureResult(context, "complete_docking");
        return context.result.transition.changed ? BT::NodeStatus::SUCCESS
                                                 : BT::NodeStatus::FAILURE;
      });

  auto tree = factory.createTreeFromText(DefaultDockingBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace robot_tasks
