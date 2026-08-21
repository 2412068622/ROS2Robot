#include "robot_tasks/station_transport_workflow.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

TEST(StationTransportWorkflowTest, PlanSubmitGate_DuplicateMissionRejects) {
  StationTransportSubmitGateRequest request;
  request.mission_id = "station_order_alpha";
  request.mission_already_active_or_queued = true;
  request.route_path = std::vector<std::string>{"receiving", "packing"};

  const auto decision = PlanStationTransportSubmitGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "station_order_alpha");
  EXPECT_EQ(
      decision.message,
      "station transport order already active or queued: station_order_alpha");
  EXPECT_TRUE(decision.route_lock_ids.empty());
}

TEST(StationTransportWorkflowTest, PlanSubmitGate_MissingCatalogRejects) {
  StationTransportSubmitGateRequest request;
  request.mission_id = "station_order_alpha";
  request.station_catalog_loaded = false;

  const auto decision = PlanStationTransportSubmitGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.message, "station transport preflight rejected: failed to load station catalog");
}

TEST(StationTransportWorkflowTest, PlanSubmitGate_PreflightRejects) {
  StationTransportSubmitGateRequest request;
  request.mission_id = "station_order_alpha";
  request.preflight.allowed = false;
  request.preflight.message = "battery insufficient";
  request.route_path = std::vector<std::string>{"receiving", "packing"};

  const auto decision = PlanStationTransportSubmitGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.message, "station transport preflight rejected: battery insufficient");
}

TEST(StationTransportWorkflowTest, PlanSubmitGate_MissingRouteRejects) {
  StationTransportSubmitGateRequest request;
  request.mission_id = "station_order_alpha";

  const auto decision = PlanStationTransportSubmitGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.message, "station transport preflight rejected: no enabled station route");
}

TEST(StationTransportWorkflowTest, PlanSubmitGate_AcceptedBuildsRouteLockIds) {
  StationTransportSubmitGateRequest request;
  request.mission_id = "station_order_alpha";
  request.route_path = std::vector<std::string>{"receiving", "packing", "storage"};
  request.traffic_intersection_locks_enabled = true;

  const auto decision = PlanStationTransportSubmitGate(request);

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "station_order_alpha");
  EXPECT_EQ(
      decision.route_lock_ids,
      (std::vector<std::string>{
          "route_node:receiving", "route_node:packing", "route_node:storage",
          "route_edge:packing__receiving", "route_edge:packing__storage"}));
}

}  // namespace
}  // namespace robot_tasks
