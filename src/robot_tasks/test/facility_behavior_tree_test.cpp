#include "robot_tasks/facility_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {
namespace {

FacilityResource MakeResource(
    const std::string& id, const std::string& type, const bool enabled = true,
    const bool exclusive = true) {
  FacilityResource resource;
  resource.id = id;
  resource.type = type;
  resource.station_id = "station_" + id;
  resource.enabled = enabled;
  resource.exclusive = exclusive;
  return resource;
}

FacilityCatalog MakeCatalog() {
  FacilityCatalog catalog;
  catalog.resources.push_back(MakeResource("door_a", "door"));
  catalog.resources.push_back(MakeResource("door_b", "door"));
  catalog.resources.push_back(MakeResource("lift_a", "elevator"));
  return catalog;
}

}  // namespace

TEST(FacilityBehaviorTreeTest, FacilityAction_ReservesSelectedResourceThroughBtAction) {
  FacilityActionBehaviorTreeInput input;
  input.catalog = MakeCatalog();
  input.resource_id = "door_a";
  input.resource_type = "door";
  input.action = "pass";
  input.request_id = "req_1";
  input.hold_after_action = false;

  const auto result = TickFacilityActionBehaviorTree(input);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.branch, "facility_action");
  EXPECT_EQ(result.resource_id, "door_a");
  EXPECT_EQ(result.mission_id, "facility_action_req_1");
  ASSERT_EQ(result.reservations.count("door_a"), 1U);
  EXPECT_EQ(result.reservations.at("door_a").status, "reserved");
  EXPECT_FALSE(result.reservations.at("door_a").hold_after_completion);
  EXPECT_EQ(result.resource_by_mission.at("facility_action_req_1"), "door_a");
}

TEST(FacilityBehaviorTreeTest, FacilityAction_RejectsUnsupportedActionBeforeReservation) {
  FacilityActionBehaviorTreeInput input;
  input.catalog = MakeCatalog();
  input.resource_id = "door_a";
  input.resource_type = "door";
  input.action = "ride";

  const auto result = TickFacilityActionBehaviorTree(input);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.message, "unsupported facility action: door.ride");
  EXPECT_TRUE(result.reservations.empty());
  EXPECT_TRUE(result.resource_by_mission.empty());
}

TEST(FacilityBehaviorTreeTest, LiftSession_ReservesAndReleasesThroughBtBranches) {
  LiftSessionBehaviorTreeInput reserve_input;
  reserve_input.lift = MakeResource("lift_a", "elevator");
  reserve_input.session_id = "session_a";
  reserve_input.requester_id = "operator_a";

  const auto reserved = TickLiftSessionBehaviorTree(reserve_input);

  ASSERT_TRUE(reserved.success);
  EXPECT_EQ(reserved.branch, "lift_reserve");
  EXPECT_EQ(reserved.lift_result.status, "lift_session");
  ASSERT_EQ(reserved.reservations.count("lift_a"), 1U);

  LiftSessionBehaviorTreeInput release_input;
  release_input.lift = MakeResource("lift_a", "elevator");
  release_input.session_id = "session_a";
  release_input.release = true;
  release_input.reservations = reserved.reservations;

  const auto released = TickLiftSessionBehaviorTree(release_input);

  ASSERT_TRUE(released.success);
  EXPECT_EQ(released.branch, "lift_release");
  EXPECT_EQ(released.lift_result.status, "available");
  EXPECT_TRUE(released.reservations.empty());
}

TEST(FacilityBehaviorTreeTest, LiftSession_RejectedReleaseDoesNotFallThroughToReserve) {
  LiftSessionBehaviorTreeInput input;
  input.lift = MakeResource("lift_a", "elevator");
  input.session_id = "session_b";
  input.release = true;
  input.reservations = {
      {"lift_a", ResourceReservation{"operator_a", "session_a", "lift_session", true}}};

  const auto result = TickLiftSessionBehaviorTree(input);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.branch, "lift_release");
  EXPECT_EQ(result.lift_result.message, "lift held by operator_a");
  ASSERT_EQ(result.reservations.count("lift_a"), 1U);
  EXPECT_EQ(result.reservations.at("lift_a").mission_id, "session_a");
}

}  // namespace robot_tasks
