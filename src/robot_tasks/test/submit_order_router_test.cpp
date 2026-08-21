#include "robot_tasks/submit_order_router.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

TEST(SubmitOrderRouterTest, BuildSubmitOrderRoute_StationTransportCliPayload_RoutesFields) {
  std::string message;
  SubmitOrderInput input;
  input.order_id = "station_alpha";
  input.order_type = "station-transport";
  input.priority = 30;
  input.payload_json =
      "pickup_station=receiving;dropoff_station=storage_a;preempt_current=true";

  const auto route = BuildSubmitOrderRoute(input, &message);

  ASSERT_TRUE(route.has_value()) << message;
  EXPECT_EQ(route->kind, SubmitOrderKind::kStationTransport);
  EXPECT_EQ(route->order_id, "station_alpha");
  EXPECT_EQ(route->priority, 30);
  EXPECT_TRUE(route->start_if_idle);
  EXPECT_TRUE(route->preempt_current);
  EXPECT_EQ(route->pickup_station, "receiving");
  EXPECT_EQ(route->dropoff_station, "storage_a");
}

TEST(SubmitOrderRouterTest, BuildSubmitOrderRoute_StationBatchJsonPayload_RoutesLists) {
  std::string message;
  SubmitOrderInput input;
  input.order_id = "batch_1";
  input.order_type = "station_order_batch";
  input.payload_json =
      R"({"order_ids":["a","b"],"pickup_station_ids":["receiving","packing"],"dropoff_station_ids":["storage_a","storage_b"],"continue_on_error":true})";

  const auto route = BuildSubmitOrderRoute(input, &message);

  ASSERT_TRUE(route.has_value()) << message;
  EXPECT_EQ(route->kind, SubmitOrderKind::kStationOrderBatch);
  EXPECT_EQ(route->batch_id, "batch_1");
  EXPECT_EQ(route->order_ids, (std::vector<std::string>{"a", "b"}));
  EXPECT_EQ(route->pickup_station_ids, (std::vector<std::string>{"receiving", "packing"}));
  EXPECT_EQ(route->dropoff_station_ids, (std::vector<std::string>{"storage_a", "storage_b"}));
  EXPECT_TRUE(route->continue_on_error);
}

TEST(SubmitOrderRouterTest, BuildSubmitOrderRoute_AllSupportedTypes_Accepted) {
  const std::vector<std::pair<std::string, SubmitOrderKind>> cases = {
      {"transport", SubmitOrderKind::kTransport},
      {"station_transport", SubmitOrderKind::kStationTransport},
      {"station_order_batch", SubmitOrderKind::kStationOrderBatch},
      {"station_sequence", SubmitOrderKind::kStationSequence},
      {"fleet_station", SubmitOrderKind::kFleetStation},
      {"business", SubmitOrderKind::kBusiness},
      {"scenario_task", SubmitOrderKind::kScenarioTask},
      {"scenario_workflow", SubmitOrderKind::kScenarioWorkflow},
      {"vda5050", SubmitOrderKind::kVda5050},
      {"picking_wave", SubmitOrderKind::kPickingWave},
      {"replenishment", SubmitOrderKind::kReplenishment},
      {"milk_run", SubmitOrderKind::kMilkRun},
  };

  for (const auto& [type, kind] : cases) {
    std::string message;
    SubmitOrderInput input;
    input.order_id = "order_1";
    input.order_type = type;

    const auto route = BuildSubmitOrderRoute(input, &message);

    ASSERT_TRUE(route.has_value()) << type << ": " << message;
    EXPECT_EQ(route->kind, kind) << type;
  }
}

TEST(SubmitOrderRouterTest, BuildSubmitOrderRoute_InvalidPayload_ReturnsMessage) {
  std::string message;
  SubmitOrderInput input;
  input.order_type = "station_transport";
  input.payload_json = "pickup_station";

  const auto route = BuildSubmitOrderRoute(input, &message);

  EXPECT_FALSE(route.has_value());
  EXPECT_EQ(message, "invalid payload_json: payload entry missing '=': pickup_station");
}

TEST(SubmitOrderRouterTest, BuildSubmitOrderRoute_UnsupportedType_ReturnsMessage) {
  std::string message;
  SubmitOrderInput input;
  input.order_type = "unknown";

  const auto route = BuildSubmitOrderRoute(input, &message);

  EXPECT_FALSE(route.has_value());
  EXPECT_NE(message.find("unsupported order_type: unknown"), std::string::npos);
}

TEST(SubmitOrderRouterTest, PlanVda5050StationOrder_ValidBuildsStationOrderFields) {
  const auto decision = PlanVda5050StationOrder(
      Vda5050StationOrderRequest{
          "order_a", "SN-1", "receiving", "packing", 42, true, false});

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.order_id, "order_a");
  EXPECT_EQ(decision.serial_number, "SN-1");
  EXPECT_EQ(decision.station_order_id, "vda5050_order_a");
  EXPECT_EQ(decision.pickup_station, "receiving");
  EXPECT_EQ(decision.dropoff_station, "packing");
  EXPECT_EQ(decision.priority, 42);
  EXPECT_TRUE(decision.start_if_idle);
  EXPECT_FALSE(decision.preempt_current);
  EXPECT_EQ(decision.message, "accepted vda5050 order order_a");
}

TEST(SubmitOrderRouterTest, PlanVda5050StationOrder_EmptyOrderRejects) {
  const auto decision = PlanVda5050StationOrder(
      Vda5050StationOrderRequest{"", "SN-1", "receiving", "packing", 0, true, false});

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "vda5050 order_id is empty");
}

TEST(SubmitOrderRouterTest, PlanVda5050StationOrder_MissingNodesRejects) {
  const auto decision = PlanVda5050StationOrder(
      Vda5050StationOrderRequest{"order_a", "SN-1", "", "packing", 0, true, false});

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "vda5050 pickup_node_id or dropoff_node_id is empty");
}

TEST(SubmitOrderRouterTest, PlanVda5050StationOrder_IdenticalNodesRejects) {
  const auto decision = PlanVda5050StationOrder(
      Vda5050StationOrderRequest{
          "order_a", "SN-1", "receiving", "receiving", 0, true, false});

  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.message, "vda5050 pickup and dropoff node are identical: receiving");
}

}  // namespace
}  // namespace robot_tasks
