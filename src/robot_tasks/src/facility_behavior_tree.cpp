#include "robot_tasks/facility_behavior_tree.hpp"

#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace robot_tasks {
namespace {

struct FacilityActionBtContext {
  FacilityActionBehaviorTreeInput input;
  const FacilityResource* selected_resource = nullptr;
  FacilityActionBehaviorTreeResult result;
};

FacilityActionBtContext MakeFacilityActionContext(
    const FacilityActionBehaviorTreeInput& input) {
  FacilityActionBtContext context;
  context.input = input;
  context.result.reservations = input.reservations;
  context.result.resource_by_mission = input.resource_by_mission;
  return context;
}

struct LiftSessionBtContext {
  LiftSessionBehaviorTreeInput input;
  LiftSessionBehaviorTreeResult result;
};

LiftSessionBtContext MakeLiftSessionContext(const LiftSessionBehaviorTreeInput& input) {
  LiftSessionBtContext context;
  context.input = input;
  context.result.reservations = input.reservations;
  return context;
}

BT::NodeStatus FailWithMessage(std::string& message, const std::string& value) {
  message = value;
  return BT::NodeStatus::FAILURE;
}

}  // namespace

std::string DefaultFacilityActionBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="FacilityActionSubflow">
  <BehaviorTree ID="FacilityActionSubflow">
    <Sequence>
      <SelectFacilityResource/>
      <CheckFacilityActionSupported/>
      <ReserveFacilityResource/>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

FacilityActionBehaviorTreeResult TickFacilityActionBehaviorTree(
    const FacilityActionBehaviorTreeInput& input) {
  auto context = MakeFacilityActionContext(input);

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction(
      "SelectFacilityResource", [&context](BT::TreeNode&) {
        const auto selection = SelectFacilityActionResource(
            context.input.catalog, context.result.reservations, context.input.resource_id,
            context.input.resource_type);
        if (!selection.success || selection.resource == nullptr) {
          return FailWithMessage(context.result.message, selection.message);
        }
        context.selected_resource = selection.resource;
        context.result.resource_id = selection.resource->id;
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleCondition(
      "CheckFacilityActionSupported", [&context](BT::TreeNode&) {
        if (context.selected_resource == nullptr) {
          return FailWithMessage(context.result.message, "facility resource was not selected");
        }
        if (FacilityActionSupported(context.selected_resource->type, context.input.action)) {
          return BT::NodeStatus::SUCCESS;
        }
        return FailWithMessage(
            context.result.message,
            "unsupported facility action: " + context.selected_resource->type + "." +
                context.input.action);
      });
  factory.registerSimpleAction(
      "ReserveFacilityResource", [&context](BT::TreeNode&) {
        if (context.selected_resource == nullptr) {
          return FailWithMessage(context.result.message, "facility resource was not selected");
        }
        context.result.mission_id = BuildFacilityActionMissionId(
            context.input.request_id, context.selected_resource->id, context.input.action);
        ReserveFacilityActionResource(
            context.result.reservations, context.result.resource_by_mission,
            *context.selected_resource, context.result.mission_id,
            context.input.hold_after_action);
        context.result.branch = "facility_action";
        context.result.message = "reserved facility resource " + context.selected_resource->id;
        return BT::NodeStatus::SUCCESS;
      });

  auto tree = factory.createTreeFromText(DefaultFacilityActionBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

std::string DefaultLiftSessionBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="LiftSessionSubflow">
  <BehaviorTree ID="LiftSessionSubflow">
    <Fallback>
      <Sequence>
        <ShouldReleaseLiftSession/>
        <ReleaseLiftSession/>
      </Sequence>
      <Sequence>
        <ShouldReserveLiftSession/>
        <ReserveLiftSession/>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
)";
}

LiftSessionBehaviorTreeResult TickLiftSessionBehaviorTree(
    const LiftSessionBehaviorTreeInput& input) {
  auto context = MakeLiftSessionContext(input);

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition(
      "ShouldReleaseLiftSession", [&context](BT::TreeNode&) {
        return context.input.release ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleCondition(
      "ShouldReserveLiftSession", [&context](BT::TreeNode&) {
        return context.input.release ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "ReleaseLiftSession", [&context](BT::TreeNode&) {
        const auto session_id = BuildLiftSessionId(
            context.input.session_id, context.input.lift.id);
        context.result.lift_result = ReleaseLiftSessionResource(
            context.result.reservations, context.input.lift.id, session_id);
        context.result.branch = "lift_release";
        return context.result.lift_result.success ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleAction(
      "ReserveLiftSession", [&context](BT::TreeNode&) {
        const auto session_id = BuildLiftSessionId(
            context.input.session_id, context.input.lift.id);
        const auto holder_id = BuildLiftSessionHolderId(
            context.input.requester_id, session_id);
        context.result.lift_result = ReserveLiftSessionResource(
            context.result.reservations, context.input.lift, session_id, holder_id,
            context.input.release_existing_for_holder);
        context.result.branch = "lift_reserve";
        return context.result.lift_result.success ? BT::NodeStatus::SUCCESS
                                                  : BT::NodeStatus::FAILURE;
      });

  auto tree = factory.createTreeFromText(DefaultLiftSessionBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace robot_tasks
