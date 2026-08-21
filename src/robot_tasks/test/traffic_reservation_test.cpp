#include "robot_tasks/traffic_reservation.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(TrafficReservationTest, RouteLockId_SortsEndpointsForSharedEdge) {
  EXPECT_EQ(RouteLockId("storage_b", "packing"), "route_edge:packing__storage_b");
  EXPECT_EQ(RouteLockId("packing", "storage_b"), "route_edge:packing__storage_b");
}

TEST(TrafficReservationTest, ParseRouteLock_ValidEdge_ReturnsEndpoints) {
  const auto parsed = ParseRouteLock("route_edge:packing__storage_b");

  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->first, "packing");
  EXPECT_EQ(parsed->second, "storage_b");
}

TEST(TrafficReservationTest, ParseRouteLock_InvalidResource_ReturnsNullopt) {
  EXPECT_FALSE(ParseRouteLock("route_node:packing").has_value());
  EXPECT_FALSE(ParseRouteLock("route_edge:packing").has_value());
  EXPECT_FALSE(ParseRouteLock("route_edge:__storage_b").has_value());
  EXPECT_FALSE(ParseRouteLock("route_edge:packing__").has_value());
}

TEST(TrafficReservationTest, BuildRouteLockIds_WithIntersectionLocks_AddsNodesAndEdges) {
  const auto lock_ids = BuildRouteLockIds({"receiving", "storage_a", "packing"}, true);

  ASSERT_EQ(lock_ids.size(), 5U);
  EXPECT_EQ(lock_ids[0], "route_node:receiving");
  EXPECT_EQ(lock_ids[1], "route_node:storage_a");
  EXPECT_EQ(lock_ids[2], "route_node:packing");
  EXPECT_EQ(lock_ids[3], "route_edge:receiving__storage_a");
  EXPECT_EQ(lock_ids[4], "route_edge:packing__storage_a");
}

TEST(TrafficReservationTest, BuildRouteLockIds_WithoutIntersectionLocks_AddsEdgesOnly) {
  const auto lock_ids = BuildRouteLockIds({"receiving", "storage_a", "packing"}, false);

  ASSERT_EQ(lock_ids.size(), 2U);
  EXPECT_EQ(lock_ids[0], "route_edge:receiving__storage_a");
  EXPECT_EQ(lock_ids[1], "route_edge:packing__storage_a");
}

TEST(TrafficReservationTest, IsTrafficBlockOwner_RecognizesOnlyTrafficBlocks) {
  EXPECT_TRUE(IsTrafficBlockOwner("traffic_block:maintenance"));
  EXPECT_FALSE(IsTrafficBlockOwner("mission_a"));
}

TEST(TrafficReservationTest, PlanTrafficBlock_UnknownStationRejects) {
  const auto decision = PlanTrafficBlock(
      TrafficBlockRequest{"receiving", "missing", "manual", false, false}, {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.resource_id, "route_edge:missing__receiving");
  EXPECT_EQ(
      decision.message, "unknown station in route block request: receiving -> missing");
}

TEST(TrafficReservationTest, PlanTrafficBlock_NoRouteRejects) {
  const auto decision = PlanTrafficBlock(
      TrafficBlockRequest{"receiving", "packing", "manual", true, false}, {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.message, "no enabled station route to block: receiving -> packing");
}

TEST(TrafficReservationTest, PlanTrafficBlock_ExistingLockRejects) {
  RouteLockMap route_locks{{"route_edge:receiving__storage_a", "mission_a"}};

  const auto decision = PlanTrafficBlock(
      TrafficBlockRequest{"receiving", "storage_a", "manual", true, true}, route_locks);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(
      decision.message,
      "route resource already reserved: route_edge:receiving__storage_a by mission_a");
}

TEST(TrafficReservationTest, PlanTrafficBlock_FreeRouteCreatesTrafficOwner) {
  const auto decision = PlanTrafficBlock(
      TrafficBlockRequest{"receiving", "storage_a", "aisle_maintenance", true, true}, {});

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_EQ(decision.resource_id, "route_edge:receiving__storage_a");
  EXPECT_EQ(decision.owner_id, "traffic_block:aisle_maintenance");
  EXPECT_EQ(decision.message, "blocked route receiving -> storage_a");
}

TEST(TrafficReservationTest, PlanTrafficUnblock_MissingLockRejects) {
  const auto decision = PlanTrafficUnblock("receiving", "storage_a", {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.resource_id, "route_edge:receiving__storage_a");
  EXPECT_EQ(decision.message, "route resource not found: route_edge:receiving__storage_a");
}

TEST(TrafficReservationTest, PlanTrafficUnblock_MissionLockRejects) {
  RouteLockMap route_locks{{"route_edge:receiving__storage_a", "mission_a"}};

  const auto decision = PlanTrafficUnblock("receiving", "storage_a", route_locks);

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.mutate_state);
  EXPECT_EQ(decision.owner_id, "mission_a");
  EXPECT_EQ(decision.message, "route resource is reserved by mission: mission_a");
}

TEST(TrafficReservationTest, PlanTrafficUnblock_TrafficBlockAllowsErase) {
  RouteLockMap route_locks{{"route_edge:receiving__storage_a", "traffic_block:manual"}};

  const auto decision = PlanTrafficUnblock("receiving", "storage_a", route_locks);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.mutate_state);
  EXPECT_EQ(decision.owner_id, "traffic_block:manual");
  EXPECT_EQ(decision.message, "unblocked route receiving -> storage_a");
}

TEST(TrafficReservationTest, PlanTrafficReservationClear_MissingRejects) {
  const auto decision = PlanTrafficReservationClear("route_edge:missing__route", {});

  EXPECT_FALSE(decision.success);
  EXPECT_FALSE(decision.erase_resource);
  EXPECT_TRUE(decision.release_mission_id.empty());
  EXPECT_EQ(decision.message, "traffic reservation not found: route_edge:missing__route");
}

TEST(TrafficReservationTest, PlanTrafficReservationClear_TrafficBlockErasesResource) {
  RouteLockMap route_locks{{"route_edge:receiving__storage_a", "traffic_block:manual"}};

  const auto decision =
      PlanTrafficReservationClear("route_edge:receiving__storage_a", route_locks);

  EXPECT_TRUE(decision.success);
  EXPECT_TRUE(decision.erase_resource);
  EXPECT_TRUE(decision.release_mission_id.empty());
  EXPECT_EQ(decision.owner_id, "traffic_block:manual");
  EXPECT_EQ(decision.message, "cleared blocked route route_edge:receiving__storage_a");
}

TEST(TrafficReservationTest, PlanTrafficReservationClear_MissionLockPlansMissionRelease) {
  RouteLockMap route_locks{{"route_edge:receiving__storage_a", "mission_a"}};

  const auto decision =
      PlanTrafficReservationClear("route_edge:receiving__storage_a", route_locks);

  EXPECT_TRUE(decision.success);
  EXPECT_FALSE(decision.erase_resource);
  EXPECT_EQ(decision.owner_id, "mission_a");
  EXPECT_EQ(decision.release_mission_id, "mission_a");
  EXPECT_EQ(
      decision.message, "cleared mission route reservation route_edge:receiving__storage_a");
}

TEST(TrafficReservationTest, PlanTrafficDeadlockDetection_IgnoresTrafficBlocksAndNodes) {
  RouteLockMap route_locks{
      {"route_edge:receiving__storage_a", "mission_a"},
      {"route_node:storage_a", "mission_b"},
      {"route_edge:packing__storage_a", "traffic_block:maintenance"},
  };

  const auto decision = PlanTrafficDeadlockDetection(route_locks);

  EXPECT_FALSE(decision.deadlocked);
  EXPECT_TRUE(decision.mission_ids.empty());
  EXPECT_EQ(decision.message, "no traffic deadlock detected");
}

TEST(TrafficReservationTest, PlanTrafficDeadlockDetection_SharedStationReportsSortedConflicts) {
  RouteLockMap route_locks{
      {"route_edge:receiving__storage_a", "mission_b"},
      {"route_edge:packing__storage_a", "mission_a"},
  };

  const auto decision = PlanTrafficDeadlockDetection(route_locks);

  EXPECT_TRUE(decision.deadlocked);
  ASSERT_EQ(decision.mission_ids.size(), 2U);
  EXPECT_EQ(decision.mission_ids[0], "mission_a");
  EXPECT_EQ(decision.mission_ids[1], "mission_b");
  ASSERT_EQ(decision.conflict_station_ids.size(), 1U);
  EXPECT_EQ(decision.conflict_station_ids[0], "storage_a");
  ASSERT_EQ(decision.conflict_resource_ids.size(), 2U);
  EXPECT_EQ(decision.conflict_resource_ids[0], "route_edge:packing__storage_a");
  EXPECT_EQ(decision.conflict_resource_ids[1], "route_edge:receiving__storage_a");
  ASSERT_EQ(decision.descriptions.size(), 1U);
  EXPECT_EQ(decision.descriptions[0], "station storage_a shared by missions mission_a, mission_b");
  EXPECT_EQ(decision.message, "traffic deadlock candidates detected");
}

TEST(TrafficReservationTest, ReserveRouteLocksForMission_FreeLocks_StoresReservation) {
  RouteLockMap route_locks;
  RouteLockReservationMap by_mission;
  std::string message;

  const bool reserved = ReserveRouteLocksForMission(
      route_locks, by_mission, "mission_a",
      {"route_node:storage_a", "route_edge:packing__storage_a"}, &message);

  EXPECT_TRUE(reserved);
  EXPECT_EQ(message, "reserved 2 route lock(s)");
  EXPECT_EQ(route_locks["route_node:storage_a"], "mission_a");
  EXPECT_EQ(route_locks["route_edge:packing__storage_a"], "mission_a");
  ASSERT_EQ(by_mission.count("mission_a"), 1U);
  EXPECT_EQ(by_mission["mission_a"].route_lock_ids.size(), 2U);
}

TEST(TrafficReservationTest, ReserveRouteLocksForMission_SameMission_IsIdempotent) {
  RouteLockMap route_locks{
      {"route_node:storage_a", "mission_a"},
  };
  RouteLockReservationMap by_mission;
  std::string message;

  const bool reserved = ReserveRouteLocksForMission(
      route_locks, by_mission, "mission_a", {"route_node:storage_a"}, &message);

  EXPECT_TRUE(reserved);
  EXPECT_EQ(route_locks["route_node:storage_a"], "mission_a");
  ASSERT_EQ(by_mission.count("mission_a"), 1U);
}

TEST(TrafficReservationTest, ReserveRouteLocksForMission_ConflictingOwner_RejectsReservation) {
  RouteLockMap route_locks{
      {"route_node:storage_a", "mission_existing"},
  };
  RouteLockReservationMap by_mission;
  std::string message;

  const bool reserved = ReserveRouteLocksForMission(
      route_locks, by_mission, "mission_new", {"route_node:storage_a"}, &message);

  EXPECT_FALSE(reserved);
  EXPECT_EQ(message, "route resource locked: route_node:storage_a by mission_existing");
  EXPECT_EQ(route_locks["route_node:storage_a"], "mission_existing");
  EXPECT_EQ(by_mission.count("mission_new"), 0U);
}

TEST(TrafficReservationTest, ReleaseRouteLocksForMission_ExistingMission_RemovesOnlyOwnedLocks) {
  RouteLockMap route_locks{
      {"route_node:storage_a", "mission_a"},
      {"route_edge:packing__storage_a", "mission_a"},
      {"route_node:packing", "mission_b"},
  };
  RouteLockReservationMap by_mission{
      {"mission_a",
       RouteLockReservation{
           "mission_a", {"route_node:storage_a", "route_edge:packing__storage_a"}}},
  };

  const bool released = ReleaseRouteLocksForMission(route_locks, by_mission, "mission_a");

  EXPECT_TRUE(released);
  EXPECT_EQ(route_locks.count("route_node:storage_a"), 0U);
  EXPECT_EQ(route_locks.count("route_edge:packing__storage_a"), 0U);
  EXPECT_EQ(route_locks["route_node:packing"], "mission_b");
  EXPECT_EQ(by_mission.count("mission_a"), 0U);
}

TEST(TrafficReservationTest, ReleaseRouteLocksForMission_MissingMission_ReturnsFalse) {
  RouteLockMap route_locks{
      {"route_node:packing", "mission_b"},
  };
  RouteLockReservationMap by_mission;

  EXPECT_FALSE(ReleaseRouteLocksForMission(route_locks, by_mission, "missing"));
  EXPECT_EQ(route_locks["route_node:packing"], "mission_b");
}

TEST(TrafficReservationTest, ReleaseRouteLocksForMission_OrphanOwnedLocks_AreRemoved) {
  RouteLockMap route_locks{
      {"route_node:receiving", "mission_a"},
      {"route_edge:receiving__storage_a", "mission_a"},
      {"route_node:packing", "mission_b"},
  };
  RouteLockReservationMap by_mission;

  const bool released = ReleaseRouteLocksForMission(route_locks, by_mission, "mission_a");

  EXPECT_TRUE(released);
  EXPECT_EQ(route_locks.count("route_node:receiving"), 0U);
  EXPECT_EQ(route_locks.count("route_edge:receiving__storage_a"), 0U);
  EXPECT_EQ(route_locks["route_node:packing"], "mission_b");
}

}  // namespace robot_tasks
