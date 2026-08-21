#include "robot_tasks/facility_reservation.hpp"

#include <gtest/gtest.h>

#include <algorithm>

namespace robot_tasks {
namespace {

FacilityResource MakeResource(
    const std::string& id, const bool enabled = true, const bool exclusive = true) {
  FacilityResource resource;
  resource.id = id;
  resource.type = "door";
  resource.station_id = "station_a";
  resource.enabled = enabled;
  resource.exclusive = exclusive;
  return resource;
}

}  // namespace

TEST(FacilityReservationTest, FacilityResourceAvailable_DisabledResource_ReturnsFalse) {
  const auto resource = MakeResource("door_a", false);
  const FacilityReservationMap reservations;

  EXPECT_FALSE(FacilityResourceAvailable(resource, reservations));
  EXPECT_EQ(FacilityResourceStatus(resource, reservations), "disabled");
}

TEST(FacilityReservationTest, FacilityResourceAvailable_ExclusiveReserved_ReturnsFalse) {
  const auto resource = MakeResource("door_a");
  const FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "reserved"}}};

  EXPECT_FALSE(FacilityResourceAvailable(resource, reservations));
  EXPECT_EQ(FacilityResourceStatus(resource, reservations), "reserved");
}

TEST(FacilityReservationTest, FacilityResourceAvailable_NonExclusiveReserved_ReturnsTrue) {
  const auto resource = MakeResource("dock_a", true, false);
  const FacilityReservationMap reservations{
      {"dock_a", ResourceReservation{"holder_a", "mission_a", "reserved"}}};

  EXPECT_TRUE(FacilityResourceAvailable(resource, reservations));
}

TEST(FacilityReservationTest, ReserveFacilityResource_FreeExclusiveResource_StoresReservation) {
  FacilityReservationMap reservations;
  const auto resource = MakeResource("door_a");

  const auto result =
      ReserveFacilityResource(reservations, resource, "holder_a", "mission_a", false);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.resource_id, "door_a");
  EXPECT_EQ(result.holder_id, "holder_a");
  EXPECT_EQ(result.status, "reserved");
  ASSERT_EQ(reservations.count("door_a"), 1U);
  EXPECT_EQ(reservations["door_a"].mission_id, "mission_a");
}

TEST(FacilityReservationTest, ReserveFacilityResource_DisabledResource_RejectsReservation) {
  FacilityReservationMap reservations;
  const auto resource = MakeResource("door_a", false);

  const auto result =
      ReserveFacilityResource(reservations, resource, "holder_a", "mission_a", false);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, "disabled");
  EXPECT_EQ(result.message, "facility resource disabled: door_a");
  EXPECT_TRUE(reservations.empty());
}

TEST(FacilityReservationTest, ReserveFacilityResource_ConflictingHolder_RejectsReservation) {
  FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_existing", "mission_existing", "reserved"}}};
  const auto resource = MakeResource("door_a");

  const auto result =
      ReserveFacilityResource(reservations, resource, "holder_new", "mission_new", false);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.holder_id, "holder_existing");
  EXPECT_EQ(result.status, "reserved");
  EXPECT_EQ(reservations["door_a"].mission_id, "mission_existing");
}

TEST(FacilityReservationTest, ReserveFacilityResource_ReleaseExistingForHolder_ClearsOldHold) {
  FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_old", "reserved"}},
      {"door_b", ResourceReservation{"holder_b", "mission_other", "reserved"}},
  };
  const auto resource = MakeResource("door_c");

  const auto result =
      ReserveFacilityResource(reservations, resource, "holder_a", "mission_new", true);

  EXPECT_TRUE(result.success);
  EXPECT_EQ(reservations.count("door_a"), 0U);
  EXPECT_EQ(reservations.count("door_b"), 1U);
  EXPECT_EQ(reservations["door_c"].mission_id, "mission_new");
}

TEST(FacilityReservationTest, ReleaseFacilityResource_MatchingHolder_ReleasesReservation) {
  FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "reserved"}}};

  const auto result = ReleaseFacilityResource(reservations, "door_a", "holder_a");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.status, "available");
  EXPECT_TRUE(reservations.empty());
}

TEST(FacilityReservationTest, ReleaseFacilityResource_WrongHolder_RejectsRelease) {
  FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "reserved"}}};

  const auto result = ReleaseFacilityResource(reservations, "door_a", "holder_b");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.holder_id, "holder_a");
  EXPECT_EQ(result.message, "facility resource held by holder_a");
  EXPECT_EQ(reservations.count("door_a"), 1U);
}

TEST(FacilityReservationTest, ReleaseResourcesForHolder_MatchingHolder_RemovesOnlyMatches) {
  FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "reserved"}},
      {"door_b", ResourceReservation{"holder_b", "mission_b", "reserved"}},
      {"door_c", ResourceReservation{"holder_a", "mission_c", "reserved"}},
  };

  const auto released = ReleaseResourcesForHolder(reservations, "holder_a");

  EXPECT_EQ(released, 2U);
  ASSERT_EQ(reservations.size(), 1U);
  EXPECT_EQ(reservations.count("door_b"), 1U);
}

TEST(FacilityReservationTest, ReservedResourceIds_ReturnsReservedIds) {
  const FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "reserved"}},
      {"door_b", ResourceReservation{"holder_b", "mission_b", "reserved"}},
  };

  const auto ids = ReservedResourceIds(reservations);

  EXPECT_EQ(ids.size(), 2U);
  EXPECT_NE(std::find(ids.begin(), ids.end(), "door_a"), ids.end());
  EXPECT_NE(std::find(ids.begin(), ids.end(), "door_b"), ids.end());
}

TEST(FacilityReservationTest, BuildFacilityResourceListProjection_FiltersAndAddsRuntimeState) {
  FacilityCatalog catalog;
  catalog.resources.push_back(FacilityResource{"door_a", "door", "station_a", true, true});
  catalog.resources.push_back(FacilityResource{"door_b", "door", "station_b", false, true});
  catalog.resources.push_back(FacilityResource{"lift_a", "elevator", "station_c", true, true});
  const FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "occupied", true}}};

  const auto projection = BuildFacilityResourceListProjection(
      catalog, reservations, FacilityResourceListRequest{false, "door"});

  EXPECT_TRUE(projection.success);
  EXPECT_EQ(projection.resource_ids, (std::vector<std::string>{"door_a"}));
  EXPECT_EQ(projection.resource_types, (std::vector<std::string>{"door"}));
  EXPECT_EQ(projection.station_ids, (std::vector<std::string>{"station_a"}));
  EXPECT_EQ(projection.enabled, (std::vector<bool>{true}));
  EXPECT_EQ(projection.available, (std::vector<bool>{false}));
  EXPECT_EQ(projection.holder_ids, (std::vector<std::string>{"holder_a"}));
  EXPECT_EQ(projection.status, (std::vector<std::string>{"occupied"}));
  EXPECT_EQ(projection.message, "loaded 1 facility resource(s)");
}

TEST(FacilityReservationTest, BuildFacilityResourceListProjection_IncludeDisabledWhenRequested) {
  FacilityCatalog catalog;
  catalog.resources.push_back(FacilityResource{"door_a", "door", "station_a", false, true});

  const auto projection = BuildFacilityResourceListProjection(
      catalog, {}, FacilityResourceListRequest{true, ""});

  EXPECT_EQ(projection.resource_ids, (std::vector<std::string>{"door_a"}));
  EXPECT_EQ(projection.enabled, (std::vector<bool>{false}));
  EXPECT_EQ(projection.available, (std::vector<bool>{false}));
  EXPECT_EQ(projection.status, (std::vector<std::string>{"disabled"}));
}

TEST(FacilityReservationTest, BuildFacilityResourceStateProjection_AddsHolderAndStatus) {
  const auto resource = MakeResource("door_a");
  const FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "occupied", true}}};

  const auto projection =
      BuildFacilityResourceStateProjection(resource, reservations, "door");

  EXPECT_TRUE(projection.success);
  EXPECT_EQ(projection.resource_id, "door_a");
  EXPECT_EQ(projection.station_id, "station_a");
  EXPECT_FALSE(projection.available);
  EXPECT_EQ(projection.holder_id, "holder_a");
  EXPECT_EQ(projection.status, "occupied");
  EXPECT_EQ(projection.message, "loaded door state door_a");
}

TEST(FacilityReservationTest, MarkResourceOccupiedForMission_HoldAfterCompletion_SetsOccupied) {
  FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "reserved", true}}};
  MissionResourceMap by_mission{{"mission_a", "door_a"}};

  EXPECT_TRUE(MarkResourceOccupiedForMission(reservations, by_mission, "mission_a"));
  EXPECT_EQ(reservations["door_a"].status, "occupied");
  EXPECT_EQ(by_mission.count("mission_a"), 1U);
}

TEST(FacilityReservationTest, MarkResourceOccupiedForMission_NoHold_ReleasesReservation) {
  FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "reserved", false}}};
  MissionResourceMap by_mission{{"mission_a", "door_a"}};

  EXPECT_TRUE(MarkResourceOccupiedForMission(reservations, by_mission, "mission_a"));
  EXPECT_TRUE(reservations.empty());
  EXPECT_TRUE(by_mission.empty());
}

TEST(FacilityReservationTest, ReleaseResourceForMission_MatchingMission_RemovesReservationAndMapping) {
  FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"holder_a", "mission_a", "reserved", true}}};
  MissionResourceMap by_mission{{"mission_a", "door_a"}};

  EXPECT_TRUE(ReleaseResourceForMission(reservations, by_mission, "mission_a"));
  EXPECT_TRUE(reservations.empty());
  EXPECT_TRUE(by_mission.empty());
}

}  // namespace robot_tasks
