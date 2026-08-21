#include "robot_tasks/submit_order_behavior_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

namespace robot_tasks {
namespace {

struct SubmitOrderBtContext {
  SubmitOrderBehaviorTreeInput input;
  SubmitOrderBehaviorTreeResult result;
};

bool RouteIs(const SubmitOrderBtContext& context, const SubmitOrderKind kind) {
  return context.result.route.has_value() && context.result.route->kind == kind;
}

BT::NodeStatus RouteCondition(
    const SubmitOrderBtContext& context, const SubmitOrderKind kind) {
  return RouteIs(context, kind) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus StageRoute(SubmitOrderBtContext& context, const std::string& branch) {
  context.result.branch = branch;
  return BT::NodeStatus::SUCCESS;
}

}  // namespace

std::string DefaultSubmitOrderBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="SubmitOrderRoute">
  <BehaviorTree ID="SubmitOrderRoute">
    <Sequence>
      <BuildSubmitOrderRoute/>
      <Fallback>
        <Sequence>
          <IsTransport/>
          <StageTransport/>
        </Sequence>
        <Sequence>
          <IsStationTransport/>
          <StageStationTransport/>
        </Sequence>
        <Sequence>
          <IsStationOrderBatch/>
          <StageStationOrderBatch/>
        </Sequence>
        <Sequence>
          <IsStationSequence/>
          <StageStationSequence/>
        </Sequence>
        <Sequence>
          <IsFleetStation/>
          <StageFleetStation/>
        </Sequence>
        <Sequence>
          <IsBusiness/>
          <StageBusiness/>
        </Sequence>
        <Sequence>
          <IsScenarioTask/>
          <StageScenarioTask/>
        </Sequence>
        <Sequence>
          <IsScenarioWorkflow/>
          <StageScenarioWorkflow/>
        </Sequence>
        <Sequence>
          <IsVda5050/>
          <StageVda5050/>
        </Sequence>
        <Sequence>
          <IsPickingWave/>
          <StagePickingWave/>
        </Sequence>
        <Sequence>
          <IsReplenishment/>
          <StageReplenishment/>
        </Sequence>
        <Sequence>
          <IsMilkRun/>
          <StageMilkRun/>
        </Sequence>
        <StageUnhandledRoute/>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

SubmitOrderBehaviorTreeResult TickSubmitOrderBehaviorTree(
    const SubmitOrderBehaviorTreeInput& input) {
  SubmitOrderBtContext context;
  context.input = input;

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleAction(
      "BuildSubmitOrderRoute", [&context](BT::TreeNode&) {
        std::string message;
        context.result.route =
            BuildSubmitOrderRoute(context.input.request, &message);
        context.result.message = message;
        return context.result.route.has_value() ? BT::NodeStatus::SUCCESS
                                                : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleCondition(
      "IsTransport", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kTransport);
      });
  factory.registerSimpleCondition(
      "IsStationTransport", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kStationTransport);
      });
  factory.registerSimpleCondition(
      "IsStationOrderBatch", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kStationOrderBatch);
      });
  factory.registerSimpleCondition(
      "IsStationSequence", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kStationSequence);
      });
  factory.registerSimpleCondition(
      "IsFleetStation", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kFleetStation);
      });
  factory.registerSimpleCondition(
      "IsBusiness", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kBusiness);
      });
  factory.registerSimpleCondition(
      "IsScenarioTask", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kScenarioTask);
      });
  factory.registerSimpleCondition(
      "IsScenarioWorkflow", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kScenarioWorkflow);
      });
  factory.registerSimpleCondition(
      "IsVda5050", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kVda5050);
      });
  factory.registerSimpleCondition(
      "IsPickingWave", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kPickingWave);
      });
  factory.registerSimpleCondition(
      "IsReplenishment", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kReplenishment);
      });
  factory.registerSimpleCondition(
      "IsMilkRun", [&context](BT::TreeNode&) {
        return RouteCondition(context, SubmitOrderKind::kMilkRun);
      });

  factory.registerSimpleAction(
      "StageTransport", [&context](BT::TreeNode&) {
        return StageRoute(context, "transport");
      });
  factory.registerSimpleAction(
      "StageStationTransport", [&context](BT::TreeNode&) {
        return StageRoute(context, "station_transport");
      });
  factory.registerSimpleAction(
      "StageStationOrderBatch", [&context](BT::TreeNode&) {
        return StageRoute(context, "station_order_batch");
      });
  factory.registerSimpleAction(
      "StageStationSequence", [&context](BT::TreeNode&) {
        return StageRoute(context, "station_sequence");
      });
  factory.registerSimpleAction(
      "StageFleetStation", [&context](BT::TreeNode&) {
        return StageRoute(context, "fleet_station");
      });
  factory.registerSimpleAction(
      "StageBusiness", [&context](BT::TreeNode&) {
        return StageRoute(context, "business");
      });
  factory.registerSimpleAction(
      "StageScenarioTask", [&context](BT::TreeNode&) {
        return StageRoute(context, "scenario_task");
      });
  factory.registerSimpleAction(
      "StageScenarioWorkflow", [&context](BT::TreeNode&) {
        return StageRoute(context, "scenario_workflow");
      });
  factory.registerSimpleAction(
      "StageVda5050", [&context](BT::TreeNode&) {
        return StageRoute(context, "vda5050");
      });
  factory.registerSimpleAction(
      "StagePickingWave", [&context](BT::TreeNode&) {
        return StageRoute(context, "picking_wave");
      });
  factory.registerSimpleAction(
      "StageReplenishment", [&context](BT::TreeNode&) {
        return StageRoute(context, "replenishment");
      });
  factory.registerSimpleAction(
      "StageMilkRun", [&context](BT::TreeNode&) {
        return StageRoute(context, "milk_run");
      });
  factory.registerSimpleAction(
      "StageUnhandledRoute", [&context](BT::TreeNode&) {
        context.result.message = "unhandled order_type route";
        return BT::NodeStatus::FAILURE;
      });

  auto tree = factory.createTreeFromText(DefaultSubmitOrderBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace robot_tasks
