#include "robot_tasks/submit_order_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

SubmitOrderBehaviorTreeInput MakeInput(
    const std::string& order_type, const std::string& payload = "") {
  SubmitOrderBehaviorTreeInput input;
  input.request.order_id = "order_1";
  input.request.order_type = order_type;
  input.request.priority = 20;
  input.request.payload_json = payload;
  return input;
}

}  // namespace

TEST(SubmitOrderBehaviorTreeTest, StationTransport_RoutesThroughBtBranch) {
  const auto result = TickSubmitOrderBehaviorTree(MakeInput(
      "station-transport",
      "pickup_station=receiving;dropoff_station=storage_a;preempt_current=true"));

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_TRUE(result.route.has_value());
  EXPECT_EQ(result.branch, "station_transport");
  EXPECT_EQ(result.route->kind, SubmitOrderKind::kStationTransport);
  EXPECT_EQ(result.route->pickup_station, "receiving");
  EXPECT_EQ(result.route->dropoff_station, "storage_a");
  EXPECT_TRUE(result.route->preempt_current);
}

TEST(SubmitOrderBehaviorTreeTest, AllSupportedTypes_SelectExpectedBranches) {
  const std::vector<std::pair<std::string, std::string>> cases = {
      {"transport", "transport"},
      {"station_transport", "station_transport"},
      {"station_order_batch", "station_order_batch"},
      {"station_sequence", "station_sequence"},
      {"fleet_station", "fleet_station"},
      {"business", "business"},
      {"scenario_task", "scenario_task"},
      {"scenario_workflow", "scenario_workflow"},
      {"vda5050", "vda5050"},
      {"picking_wave", "picking_wave"},
      {"replenishment", "replenishment"},
      {"milk_run", "milk_run"},
  };

  for (const auto& [order_type, branch] : cases) {
    const auto result = TickSubmitOrderBehaviorTree(MakeInput(order_type));

    ASSERT_TRUE(result.success) << order_type << ": " << result.message;
    EXPECT_EQ(result.branch, branch) << order_type;
    ASSERT_TRUE(result.route.has_value()) << order_type;
  }
}

TEST(SubmitOrderBehaviorTreeTest, InvalidPayload_FailsBeforeRouting) {
  const auto result =
      TickSubmitOrderBehaviorTree(MakeInput("station_transport", "pickup_station"));

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.route.has_value());
  EXPECT_EQ(result.branch, "");
  EXPECT_EQ(result.message, "invalid payload_json: payload entry missing '=': pickup_station");
}

TEST(SubmitOrderBehaviorTreeTest, UnsupportedType_FailsWithRouterMessage) {
  const auto result = TickSubmitOrderBehaviorTree(MakeInput("unknown"));

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.route.has_value());
  EXPECT_NE(result.message.find("unsupported order_type: unknown"), std::string::npos);
}

}  // namespace robot_tasks
