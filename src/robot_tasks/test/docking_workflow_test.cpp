#include "robot_tasks/docking_workflow.hpp"

#include <gtest/gtest.h>

namespace robot_tasks {

TEST(DockingWorkflowTest, CompleteDockingSubflow_MappedMission_SetsChargingAndResourceStatus) {
  DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", true, 0}}};
  const DockingMissionMap dock_by_mission{{"mission_a", "dock_a"}};
  FacilityReservationMap reservations{
      {"charger_a", ResourceReservation{"mission_a", "mission_a", "APPROACHING", true}}};
  const MissionResourceMap resource_by_mission{{"mission_a", "charger_a"}};

  const auto transition = CompleteDockingSubflow(
      dock_states, dock_by_mission, reservations, resource_by_mission, "mission_a");

  ASSERT_TRUE(transition.changed);
  EXPECT_EQ(transition.previous_state, "APPROACHING");
  EXPECT_EQ(transition.state, "CHARGING");
  EXPECT_EQ(transition.event_state, "CHARGING");
  EXPECT_EQ(transition.event_message, "docked and charging at dock_a");
  EXPECT_EQ(dock_states["dock_a"].state, "CHARGING");
  EXPECT_EQ(reservations["charger_a"].status, "CHARGING");
}

TEST(DockingWorkflowTest, CompleteDockingSubflow_UnknownMission_DoesNotMutateState) {
  DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", true, 0}}};
  const DockingMissionMap dock_by_mission{{"mission_a", "dock_a"}};
  FacilityReservationMap reservations{
      {"charger_a", ResourceReservation{"mission_a", "mission_a", "APPROACHING", true}}};
  const MissionResourceMap resource_by_mission{{"mission_a", "charger_a"}};

  const auto transition = CompleteDockingSubflow(
      dock_states, dock_by_mission, reservations, resource_by_mission, "mission_missing");

  EXPECT_FALSE(transition.changed);
  EXPECT_EQ(dock_states["dock_a"].state, "APPROACHING");
  EXPECT_EQ(reservations["charger_a"].status, "APPROACHING");
}

TEST(DockingWorkflowTest, StageDockingContactRetry_FirstSimulatedFailure_StagesRetry) {
  DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", false, 0}}};
  const DockingMissionMap dock_by_mission{{"mission_a", "dock_a"}};
  FacilityReservationMap reservations{
      {"charger_a", ResourceReservation{"mission_a", "mission_a", "APPROACHING", true}}};
  const MissionResourceMap resource_by_mission{{"mission_a", "charger_a"}};

  const auto transition = StageDockingContactRetry(
      dock_states, dock_by_mission, reservations, resource_by_mission, "mission_a");

  ASSERT_TRUE(transition.changed);
  EXPECT_EQ(transition.previous_state, "APPROACHING");
  EXPECT_EQ(transition.state, "FAILED");
  EXPECT_EQ(transition.event_state, "DOCKING_CONTACT_FAILED");
  EXPECT_EQ(dock_states["dock_a"].contact_attempts, 1);
  EXPECT_TRUE(dock_states["dock_a"].simulate_contact_success);
  EXPECT_EQ(reservations["charger_a"].status, "DOCKING_CONTACT_FAILED");
}

TEST(DockingWorkflowTest, StageDockingContactRetry_AlreadyRetried_DoesNotStageAgain) {
  DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "mission_a", "FAILED", true, 1}}};
  const DockingMissionMap dock_by_mission{{"mission_a", "dock_a"}};
  FacilityReservationMap reservations{
      {"charger_a", ResourceReservation{"mission_a", "mission_a", "DOCKING_CONTACT_FAILED", true}}};
  const MissionResourceMap resource_by_mission{{"mission_a", "charger_a"}};

  const auto transition = StageDockingContactRetry(
      dock_states, dock_by_mission, reservations, resource_by_mission, "mission_a");

  EXPECT_FALSE(transition.changed);
  EXPECT_EQ(dock_states["dock_a"].contact_attempts, 1);
  EXPECT_EQ(reservations["charger_a"].status, "DOCKING_CONTACT_FAILED");
}

TEST(DockingWorkflowTest, MarkDockingSubflowTerminal_MappedMission_ErasesMissionMapping) {
  DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", true, 0}}};
  DockingMissionMap dock_by_mission{{"mission_a", "dock_a"}};

  const auto transition = MarkDockingSubflowTerminal(
      dock_states, dock_by_mission, "mission_a", "CANCELED", "docking mission canceled");

  ASSERT_TRUE(transition.changed);
  EXPECT_EQ(transition.previous_state, "APPROACHING");
  EXPECT_EQ(transition.event_state, "CANCELED");
  EXPECT_TRUE(transition.event_recoverable);
  EXPECT_EQ(dock_states["dock_a"].state, "CANCELED");
  EXPECT_TRUE(dock_by_mission.empty());
}

TEST(DockingWorkflowTest, MarkDockingSubflowTerminal_FailedMission_IsNotRecoverable) {
  DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "mission_a", "APPROACHING", true, 0}}};
  DockingMissionMap dock_by_mission{{"mission_a", "dock_a"}};

  const auto transition = MarkDockingSubflowTerminal(
      dock_states, dock_by_mission, "mission_a", "FAILED", "docking failed");

  ASSERT_TRUE(transition.changed);
  EXPECT_FALSE(transition.event_recoverable);
}

TEST(DockingWorkflowTest, PlanDockReturn_LoadFailed_SetsErrorResponse) {
  DockReturnRequest request;
  request.dock_profile_available = false;
  request.load_error_message = "failed to load dock mission profile: dock.yaml";

  const auto decision = PlanDockReturn(request);

  EXPECT_EQ(decision.branch, "load_failed");
  EXPECT_TRUE(decision.set_error_state);
  EXPECT_FALSE(decision.response_success);
  EXPECT_EQ(decision.response_message, request.load_error_message);
}

TEST(DockingWorkflowTest, PlanDockReturn_AlreadyRequested_OnlyReportsSuccess) {
  DockReturnRequest request;
  request.dock_profile_available = true;
  request.dock_already_requested_or_running = true;
  request.mission_active = true;

  const auto decision = PlanDockReturn(request);

  EXPECT_EQ(decision.branch, "already_requested");
  EXPECT_TRUE(decision.response_success);
  EXPECT_FALSE(decision.queue_dock_mission);
  EXPECT_FALSE(decision.cancel_active_mission);
  EXPECT_FALSE(decision.dispatch_dock_mission);
}

TEST(DockingWorkflowTest, PlanDockReturn_ActiveMission_QueuesDockAndCancelsActive) {
  DockReturnRequest request;
  request.dock_profile_available = true;
  request.mission_active = true;

  const auto decision = PlanDockReturn(request);

  EXPECT_EQ(decision.branch, "queue_then_cancel_active");
  EXPECT_TRUE(decision.response_success);
  EXPECT_TRUE(decision.queue_dock_mission);
  EXPECT_TRUE(decision.cancel_active_mission);
  EXPECT_FALSE(decision.dispatch_dock_mission);
}

TEST(DockingWorkflowTest, PlanDockReturn_IdleMission_DispatchesImmediately) {
  DockReturnRequest request;
  request.dock_profile_available = true;

  const auto decision = PlanDockReturn(request);

  EXPECT_EQ(decision.branch, "dispatch_now");
  EXPECT_FALSE(decision.queue_dock_mission);
  EXPECT_FALSE(decision.cancel_active_mission);
  EXPECT_TRUE(decision.dispatch_dock_mission);
}

TEST(DockingWorkflowTest, PlanDockingRequestGate_DisabledDockRejectsWithState) {
  DockingRequestGateRequest request;
  request.dock_id = "dock_a";
  request.dock_enabled = false;
  request.dock_available = true;
  request.dock_state = "DISABLED";

  const auto decision = PlanDockingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.state, "DISABLED");
  EXPECT_EQ(decision.message, "dock disabled: dock_a");
}

TEST(DockingWorkflowTest, PlanDockingRequestGate_UnavailableDockRejectsWithState) {
  DockingRequestGateRequest request;
  request.dock_id = "dock_a";
  request.dock_enabled = true;
  request.dock_available = false;
  request.dock_state = "CHARGING";

  const auto decision = PlanDockingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.state, "CHARGING");
  EXPECT_EQ(decision.message, "dock unavailable: dock_a");
}

TEST(DockingWorkflowTest, PlanDockingRequestGate_InvalidReferencesUseReferenceMessage) {
  DockingRequestGateRequest request;
  request.dock_id = "dock_a";
  request.dock_references_valid = false;
  request.dock_reference_message = "dock approach station not found: dock_entry";

  const auto decision = PlanDockingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.message, "dock approach station not found: dock_entry");
}

TEST(DockingWorkflowTest, PlanDockingRequestGate_DuplicateMissionRejectsBeforePreflight) {
  DockingRequestGateRequest request;
  request.dock_id = "dock_a";
  request.mission_id = "docking_request_alpha";
  request.mission_already_active_or_queued = true;
  request.preflight.allowed = false;
  request.preflight.message = "would fail if evaluated first";

  const auto decision = PlanDockingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "docking_request_alpha");
  EXPECT_EQ(decision.message, "docking mission already active or queued: docking_request_alpha");
}

TEST(DockingWorkflowTest, PlanDockingRequestGate_PreflightRejectsWithRejectedState) {
  DockingRequestGateRequest request;
  request.dock_id = "dock_a";
  request.mission_id = "docking_request_alpha";
  request.preflight.allowed = false;
  request.preflight.message = "battery insufficient";

  const auto decision = PlanDockingRequestGate(request);

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "docking_request_alpha");
  EXPECT_EQ(decision.state, "REJECTED");
  EXPECT_EQ(decision.message, "docking preflight rejected: battery insufficient");
}

TEST(DockingWorkflowTest, PlanDockingRequestGate_AcceptsApproach) {
  DockingRequestGateRequest request;
  request.dock_id = "dock_a";
  request.mission_id = "docking_request_alpha";

  const auto decision = PlanDockingRequestGate(request);

  EXPECT_TRUE(decision.accepted);
  EXPECT_EQ(decision.mission_id, "docking_request_alpha");
  EXPECT_EQ(decision.state, "APPROACHING");
  EXPECT_TRUE(decision.message.empty());
}

TEST(DockingWorkflowTest, PlanUndocking_NoRequestId_UsesCurrentDockMission) {
  const DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "docking_request_alpha", "CHARGING"}}};
  const FacilityReservationMap reservations{
      {"charger_a", ResourceReservation{"docking_request_alpha", "docking_request_alpha",
                                        "CHARGING", true}}};

  const auto decision = PlanUndocking(
      UndockingRequest{"dock_a", "charger_a", "", true}, dock_states, reservations);

  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.mission_id, "docking_request_alpha");
  EXPECT_EQ(decision.release_mission_id, "docking_request_alpha");
  EXPECT_EQ(decision.previous_state, "CHARGING");
  EXPECT_EQ(decision.state, "UNDOCKED");
  EXPECT_TRUE(decision.release_charger);
}

TEST(DockingWorkflowTest, ApplyUndockingDecision_ReleasesOnlyMatchingMissionResource) {
  DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "docking_request_alpha", "CHARGING"}}};
  FacilityReservationMap reservations{
      {"charger_a", ResourceReservation{"docking_request_alpha", "docking_request_alpha",
                                        "CHARGING", true}},
      {"charger_b", ResourceReservation{"mission_b", "mission_b", "reserved", true}}};
  MissionResourceMap resource_by_mission{
      {"docking_request_alpha", "charger_a"},
      {"mission_other", "charger_a"},
      {"mission_b", "charger_b"}};

  const auto decision = PlanUndocking(
      UndockingRequest{"dock_a", "charger_a", "", true}, dock_states, reservations);
  ApplyUndockingDecision(decision, "charger_a", dock_states, reservations, resource_by_mission);

  EXPECT_EQ(dock_states["dock_a"].state, "UNDOCKED");
  EXPECT_EQ(dock_states["dock_a"].mission_id, "docking_request_alpha");
  EXPECT_EQ(reservations.count("charger_a"), 0U);
  EXPECT_EQ(reservations.count("charger_b"), 1U);
  EXPECT_EQ(resource_by_mission.count("docking_request_alpha"), 0U);
  EXPECT_EQ(resource_by_mission.count("mission_other"), 1U);
  EXPECT_EQ(resource_by_mission.count("mission_b"), 1U);
}

TEST(DockingWorkflowTest, ApplyUndockingDecision_RequestIdOverride_ReleasesCurrentDockHolder) {
  DockingStateMap dock_states{
      {"dock_a", DockingRuntimeState{"dock_a", "docking_request_alpha", "CHARGING"}}};
  FacilityReservationMap reservations{
      {"charger_a", ResourceReservation{"docking_request_alpha", "docking_request_alpha",
                                        "CHARGING", true}}};
  MissionResourceMap resource_by_mission{{"docking_request_alpha", "charger_a"}};

  const auto decision = PlanUndocking(
      UndockingRequest{"dock_a", "charger_a", "manual", true}, dock_states, reservations);
  ApplyUndockingDecision(decision, "charger_a", dock_states, reservations, resource_by_mission);

  EXPECT_EQ(decision.mission_id, "undocking_request_manual");
  EXPECT_EQ(decision.release_mission_id, "docking_request_alpha");
  EXPECT_EQ(dock_states["dock_a"].mission_id, "undocking_request_manual");
  EXPECT_EQ(reservations.count("charger_a"), 0U);
  EXPECT_EQ(resource_by_mission.count("docking_request_alpha"), 0U);
}

}  // namespace robot_tasks
