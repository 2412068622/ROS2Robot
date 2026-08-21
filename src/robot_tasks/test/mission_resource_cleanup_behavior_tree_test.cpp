#include "robot_tasks/mission_resource_cleanup_behavior_tree.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(MissionResourceCleanupBehaviorTreeTest, ExistingResources_ReleaseThroughBtBranch) {
  FacilityReservationMap facility_reservations;
  MissionResourceMap resource_by_mission;
  facility_reservations["door_a"] = ResourceReservation{"holder", "mission_a", "reserved"};
  resource_by_mission["mission_a"] = "door_a";
  RouteLockMap route_locks;
  RouteLockReservationMap route_locks_by_mission;
  std::string message;
  ASSERT_TRUE(ReserveRouteLocksForMission(
      route_locks, route_locks_by_mission, "mission_a", {"route_node:receiving"}, &message));

  const auto result = TickMissionResourceCleanupBehaviorTree(
      MissionResourceCleanupBehaviorTreeInput{
          facility_reservations, resource_by_mission, route_locks, route_locks_by_mission,
          "mission_a"});

  EXPECT_TRUE(result.success);
  EXPECT_EQ("released", result.branch);
  EXPECT_TRUE(result.cleanup.facility_resource_released);
  EXPECT_TRUE(result.cleanup.route_locks_released);
  EXPECT_TRUE(facility_reservations.empty());
  EXPECT_TRUE(resource_by_mission.empty());
  EXPECT_TRUE(route_locks.empty());
  EXPECT_TRUE(route_locks_by_mission.empty());
}

TEST(MissionResourceCleanupBehaviorTreeTest, MissingMission_TakesNoopBranch) {
  FacilityReservationMap facility_reservations;
  MissionResourceMap resource_by_mission;
  RouteLockMap route_locks;
  RouteLockReservationMap route_locks_by_mission;

  const auto result = TickMissionResourceCleanupBehaviorTree(
      MissionResourceCleanupBehaviorTreeInput{
          facility_reservations, resource_by_mission, route_locks, route_locks_by_mission,
          "missing"});

  EXPECT_TRUE(result.success);
  EXPECT_EQ("noop", result.branch);
  EXPECT_FALSE(result.cleanup.facility_resource_released);
  EXPECT_FALSE(result.cleanup.route_locks_released);
}

}  // namespace robot_tasks
