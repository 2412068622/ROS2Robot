#include "robot_tasks/station_sequence_behavior_tree.hpp"

#include <cstddef>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

namespace robot_tasks {
namespace {

struct StationSequenceBtContext {
  StationSequenceBehaviorTreeInput input;
  StationSequenceBehaviorTreeResult result;
};

StationSequenceBtContext MakeStationSequenceContext(
    const StationSequenceBehaviorTreeInput& input) {
  StationSequenceBtContext context;
  context.input = input;
  context.result.message = "station sequence accepted";
  return context;
}

MissionPreflightResult DefaultAllowedPreflight() {
  MissionPreflightResult preflight;
  preflight.allowed = true;
  preflight.message = "allowed";
  return preflight;
}

}  // namespace

std::string DefaultStationSequenceBehaviorTreeXml() {
  return R"(
<root BTCPP_format="4" main_tree_to_execute="StationSequenceSubflow">
  <BehaviorTree ID="StationSequenceSubflow">
    <Sequence>
      <CheckStationSequenceConflicts/>
      <CheckStationSequencePreflight/>
      <BuildStationSequenceQueue/>
    </Sequence>
  </BehaviorTree>
</root>
)";
}

StationSequenceBehaviorTreeResult TickStationSequenceBehaviorTree(
    const StationSequenceBehaviorTreeInput& input) {
  auto context = MakeStationSequenceContext(input);

  BT::BehaviorTreeFactory factory;
  factory.registerSimpleCondition(
      "CheckStationSequenceConflicts", [&context](BT::TreeNode&) {
        const auto result = CheckStationSequenceConflicts(
            context.input.legs, context.input.active_or_queued_mission_ids);
        context.result.message = result.message;
        return result.accepted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      });
  factory.registerSimpleCondition(
      "CheckStationSequencePreflight", [&context](BT::TreeNode&) {
        for (std::size_t index = 0; index < context.input.legs.size(); ++index) {
          const auto preflight = index < context.input.preflights.size()
                                     ? context.input.preflights[index]
                                     : DefaultAllowedPreflight();
          const auto result =
              CheckStationSequenceLegPreflight(context.input.legs[index], preflight);
          if (!result.accepted) {
            context.result.message = result.message;
            return BT::NodeStatus::FAILURE;
          }
        }
        return BT::NodeStatus::SUCCESS;
      });
  factory.registerSimpleAction(
      "BuildStationSequenceQueue", [&context](BT::TreeNode&) {
        context.result.mission_ids = StationSequenceMissionIds(context.input.legs);
        context.result.queued_missions = BuildStationSequenceQueuedMissions(
            context.input.legs, context.input.priority, context.input.first_sequence);
        context.result.branch = "station_sequence_queue";
        context.result.message =
            BuildStationSequenceQueuedMessage(context.result.queued_missions.size());
        return BT::NodeStatus::SUCCESS;
      });

  auto tree = factory.createTreeFromText(DefaultStationSequenceBehaviorTreeXml());
  context.result.success = tree.tickOnce() == BT::NodeStatus::SUCCESS;
  return context.result;
}

}  // namespace robot_tasks
