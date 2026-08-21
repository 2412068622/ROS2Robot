#include "robot_tasks/mission_resource_cleanup.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(MissionResourceCleanupTest, ReleaseMissionResources_ReleasesFacilityAndRouteLocks) {
  FacilityReservationMap facility_reservations;
  MissionResourceMap resource_by_mission;
  facility_reservations["door_a"] = ResourceReservation{"holder", "mission_a", "reserved"};
  resource_by_mission["mission_a"] = "door_a";
  RouteLockMap route_locks;
  RouteLockReservationMap route_locks_by_mission;
  std::string message;
  ASSERT_TRUE(ReserveRouteLocksForMission(
      route_locks, route_locks_by_mission, "mission_a",
      {"route_node:receiving", "route_edge:receiving__packing"}, &message));

  const auto result = ReleaseMissionResources(
      facility_reservations, resource_by_mission, route_locks, route_locks_by_mission,
      "mission_a");

  EXPECT_TRUE(result.facility_resource_released);
  EXPECT_TRUE(result.route_locks_released);
  EXPECT_TRUE(facility_reservations.empty());
  EXPECT_TRUE(resource_by_mission.empty());
  EXPECT_TRUE(route_locks.empty());
  EXPECT_TRUE(route_locks_by_mission.empty());
}

TEST(MissionResourceCleanupTest, ReleaseMissionResources_RouteOnlyStillReleasesRouteLocks) {
  FacilityReservationMap facility_reservations;
  MissionResourceMap resource_by_mission;
  RouteLockMap route_locks;
  RouteLockReservationMap route_locks_by_mission;
  std::string message;
  ASSERT_TRUE(ReserveRouteLocksForMission(
      route_locks, route_locks_by_mission, "mission_a", {"route_node:receiving"},
      &message));

  const auto result = ReleaseMissionResources(
      facility_reservations, resource_by_mission, route_locks, route_locks_by_mission,
      "mission_a");

  EXPECT_FALSE(result.facility_resource_released);
  EXPECT_TRUE(result.route_locks_released);
  EXPECT_TRUE(route_locks.empty());
  EXPECT_TRUE(route_locks_by_mission.empty());
}

TEST(MissionResourceCleanupTest, ReleaseMissionResources_MissingMissionReportsNoRelease) {
  FacilityReservationMap facility_reservations;
  MissionResourceMap resource_by_mission;
  RouteLockMap route_locks;
  RouteLockReservationMap route_locks_by_mission;

  const auto result = ReleaseMissionResources(
      facility_reservations, resource_by_mission, route_locks, route_locks_by_mission,
      "missing");

  EXPECT_FALSE(result.facility_resource_released);
  EXPECT_FALSE(result.route_locks_released);
}

}  // namespace robot_tasks
