#include "robot_tasks/facility_workflow.hpp"

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

TEST(FacilityWorkflowTest, FacilityActionSupported_KnownResourceTypes_UsesExpectedActions) {
  EXPECT_TRUE(FacilityActionSupported("door", "open"));
  EXPECT_TRUE(FacilityActionSupported("door", "pass"));
  EXPECT_FALSE(FacilityActionSupported("door", "ride"));
  EXPECT_TRUE(FacilityActionSupported("elevator", "ride"));
  EXPECT_FALSE(FacilityActionSupported("elevator", "open"));
  EXPECT_TRUE(FacilityActionSupported("charger", "reserve"));
  EXPECT_FALSE(FacilityActionSupported("charger", "use"));
}

TEST(FacilityWorkflowTest, SelectFacilityActionResource_ExplicitAvailableResource_ReturnsResource) {
  const auto catalog = MakeCatalog();
  const FacilityReservationMap reservations;

  const auto selection = SelectFacilityActionResource(catalog, reservations, "door_a", "door");

  ASSERT_TRUE(selection.success);
  ASSERT_NE(selection.resource, nullptr);
  EXPECT_EQ(selection.resource->id, "door_a");
}

TEST(FacilityWorkflowTest, SelectFacilityActionResource_TypeMismatch_ReturnsMessage) {
  const auto catalog = MakeCatalog();
  const FacilityReservationMap reservations;

  const auto selection = SelectFacilityActionResource(catalog, reservations, "door_a", "elevator");

  EXPECT_FALSE(selection.success);
  EXPECT_EQ(selection.message, "facility resource type mismatch: door_a");
}

TEST(FacilityWorkflowTest, SelectFacilityActionResource_ExplicitUnavailable_ReturnsMessage) {
  const auto catalog = MakeCatalog();
  const FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"mission_a", "mission_a", "reserved", true}}};

  const auto selection = SelectFacilityActionResource(catalog, reservations, "door_a", "door");

  EXPECT_FALSE(selection.success);
  EXPECT_EQ(selection.message, "facility resource unavailable: door_a");
}

TEST(FacilityWorkflowTest, SelectFacilityActionResource_ByType_SkipsReservedResource) {
  const auto catalog = MakeCatalog();
  const FacilityReservationMap reservations{
      {"door_a", ResourceReservation{"mission_a", "mission_a", "reserved", true}}};

  const auto selection = SelectFacilityActionResource(catalog, reservations, "", "door");

  ASSERT_TRUE(selection.success);
  ASSERT_NE(selection.resource, nullptr);
  EXPECT_EQ(selection.resource->id, "door_b");
}

TEST(FacilityWorkflowTest, BuildFacilityActionMissionId_EmptyRequestId_UsesResourceAndAction) {
  EXPECT_EQ(
      BuildFacilityActionMissionId("", "door_a", "open"), "facility_action_door_a_open");
  EXPECT_EQ(
      BuildFacilityActionMissionId("operator_1", "door_a", "open"),
      "facility_action_operator_1");
}

TEST(FacilityWorkflowTest, ReserveFacilityActionResource_StoresHoldPolicyAndMissionMapping) {
  FacilityReservationMap reservations;
  MissionResourceMap resource_by_mission;
  const auto resource = MakeResource("door_a", "door");

  ReserveFacilityActionResource(
      reservations, resource_by_mission, resource, "mission_a", false);

  ASSERT_EQ(reservations.count("door_a"), 1U);
  EXPECT_EQ(reservations["door_a"].holder_id, "mission_a");
  EXPECT_EQ(reservations["door_a"].mission_id, "mission_a");
  EXPECT_EQ(reservations["door_a"].status, "reserved");
  EXPECT_FALSE(reservations["door_a"].hold_after_completion);
  EXPECT_EQ(resource_by_mission["mission_a"], "door_a");
}

TEST(FacilityWorkflowTest, BuildLiftSessionId_EmptyRequest_UsesLiftId) {
  EXPECT_EQ(BuildLiftSessionId("", "lift_a"), "lift_lift_a");
  EXPECT_EQ(BuildLiftSessionId("session_1", "lift_a"), "session_1");
}

TEST(FacilityWorkflowTest, BuildLiftSessionHolderId_EmptyRequester_UsesSessionId) {
  EXPECT_EQ(BuildLiftSessionHolderId("", "session_1"), "session_1");
  EXPECT_EQ(BuildLiftSessionHolderId("operator_a", "session_1"), "operator_a");
}

TEST(FacilityWorkflowTest, ReserveLiftSessionResource_FreeLift_StoresSession) {
  FacilityReservationMap reservations;
  const auto lift = MakeResource("lift_a", "elevator");

  const auto result =
      ReserveLiftSessionResource(reservations, lift, "session_a", "operator_a", false);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.lift_id, "lift_a");
  EXPECT_EQ(result.session_id, "session_a");
  EXPECT_EQ(result.status, "lift_session");
  ASSERT_EQ(reservations.count("lift_a"), 1U);
  EXPECT_EQ(reservations["lift_a"].holder_id, "operator_a");
  EXPECT_EQ(reservations["lift_a"].mission_id, "session_a");
}

TEST(FacilityWorkflowTest, ReserveLiftSessionResource_ConflictingHolder_RejectsSession) {
  FacilityReservationMap reservations{
      {"lift_a", ResourceReservation{"operator_existing", "session_existing", "lift_session", true}}};
  const auto lift = MakeResource("lift_a", "elevator");

  const auto result =
      ReserveLiftSessionResource(reservations, lift, "session_new", "operator_new", false);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, "lift_session");
  EXPECT_EQ(result.message, "lift already reserved by operator_existing");
  EXPECT_EQ(reservations["lift_a"].mission_id, "session_existing");
}

TEST(FacilityWorkflowTest, ReserveLiftSessionResource_ReleaseExistingForHolder_ClearsOldHold) {
  FacilityReservationMap reservations{
      {"lift_old", ResourceReservation{"operator_a", "session_old", "lift_session", true}},
      {"lift_other", ResourceReservation{"operator_b", "session_other", "lift_session", true}},
  };
  const auto lift = MakeResource("lift_a", "elevator");

  const auto result =
      ReserveLiftSessionResource(reservations, lift, "session_new", "operator_a", true);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(reservations.count("lift_old"), 0U);
  EXPECT_EQ(reservations.count("lift_other"), 1U);
  ASSERT_EQ(reservations.count("lift_a"), 1U);
  EXPECT_EQ(reservations["lift_a"].mission_id, "session_new");
}

TEST(FacilityWorkflowTest, ReleaseLiftSessionResource_MissingLift_ReturnsAvailable) {
  FacilityReservationMap reservations;

  const auto result = ReleaseLiftSessionResource(reservations, "lift_a", "session_a");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.lift_id, "lift_a");
  EXPECT_EQ(result.session_id, "session_a");
  EXPECT_EQ(result.status, "available");
  EXPECT_EQ(result.message, "lift already available: lift_a");
}

TEST(FacilityWorkflowTest, ReleaseLiftSessionResource_WrongSession_RejectsRelease) {
  FacilityReservationMap reservations{
      {"lift_a", ResourceReservation{"operator_a", "session_a", "lift_session", true}}};

  const auto result = ReleaseLiftSessionResource(reservations, "lift_a", "session_b");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, "lift_session");
  EXPECT_EQ(result.message, "lift held by operator_a");
  EXPECT_EQ(reservations.count("lift_a"), 1U);
}

TEST(FacilityWorkflowTest, ReleaseLiftSessionResource_MatchingSession_ReleasesLift) {
  FacilityReservationMap reservations{
      {"lift_a", ResourceReservation{"operator_a", "session_a", "lift_session", true}}};

  const auto result = ReleaseLiftSessionResource(reservations, "lift_a", "session_a");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.status, "available");
  EXPECT_EQ(result.message, "released lift session session_a");
  EXPECT_TRUE(reservations.empty());
}

}  // namespace robot_tasks
